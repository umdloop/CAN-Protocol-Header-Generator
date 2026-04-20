#include <generator.hpp>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <cctype>
#include <tuple>

static std::string toIdentifier(const std::string& name) {
    std::string result;
    result.reserve(name.size());
    for (char c : name)
        result += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    if (!result.empty() && std::isdigit(static_cast<unsigned char>(result[0])))
        result.insert(0, 1, '_');
    return result;
}

static std::string toCamelCase(const std::string& name, bool firstUpper = true) {
    std::string result;
    bool nextUpper = firstUpper;
    for (char c : name) {
        if (c == '_') {
            nextUpper = true;
        } else {
            result += nextUpper ? static_cast<char>(std::toupper(static_cast<unsigned char>(c)))
                                : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            nextUpper = false;
        }
    }
    return result;
}

static std::string cppType(const CppCAN::CANSignal& sig) {
    if (sig.length() == 1) return "bool";
    bool isSigned = (sig.signedness() == CppCAN::CANSignal::Signed);
    unsigned int len = sig.length();
    if (!isSigned) {
        if (len <= 8)  return "uint8_t";
        if (len <= 16) return "uint16_t";
        if (len <= 32) return "uint32_t";
        return "uint64_t";
    } else {
        if (len <= 8)  return "int8_t";
        if (len <= 16) return "int16_t";
        if (len <= 32) return "int32_t";
        return "int64_t";
    }
}

static bool splitPortSuffix(const std::string& name, std::string& stem, int& port) {
    auto pos = name.rfind('_');
    if (pos == std::string::npos || pos + 1 >= name.size()) return false;
    const std::string suffix = name.substr(pos + 1);
    if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(), ::isdigit)) return false;
    stem = name.substr(0, pos);
    port = std::stoi(suffix);
    return true;
}

static const char* HEADER_BOILERPLATE = R"(#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <optional>

namespace Protocol {

template<int StartBit, int Length, typename UInt>
inline void encode_field(uint8_t* data, UInt value) {
    using U64 = uint64_t;
    using Uns = std::make_unsigned_t<UInt>;
    U64 word = 0;
    std::memcpy(&word, data, 8);
    constexpr U64 mask = (Length == 64) ? ~U64{0} : ((U64{1} << Length) - 1);
    word = (word & ~(mask << StartBit)) | ((static_cast<U64>(static_cast<Uns>(value)) & mask) << StartBit);
    std::memcpy(data, &word, 8);
}

template<int StartBit, int Length, typename T = uint64_t>
inline T decode_field(const uint8_t* data) {
    using U64 = uint64_t;
    U64 word = 0;
    std::memcpy(&word, data, 8);
    constexpr U64 mask = (Length == 64) ? ~U64{0} : ((U64{1} << Length) - 1);
    U64 raw = (word >> StartBit) & mask;
    if constexpr (std::is_signed_v<T> && Length < 64) {
        if (raw >> (Length - 1)) raw |= ~mask;
        return static_cast<T>(static_cast<int64_t>(raw));
    }
    return static_cast<T>(raw);
}

struct CanFrame {
    uint32_t id{};
    uint8_t dlc{};
    std::array<uint8_t, 8> data{};
    bool is_extended{};
};

)";

struct FieldMeta {
    std::string stem;
    std::string api_name;
    unsigned int start_bit;
    unsigned int length;
    std::string type;
};

struct Support {
    std::string pcb;
    uint32_t id;
    int base_mux;
    int num_ports;
    int mux_start;
    int mux_len;
    int dlc;
};

struct GroupMeta {
    std::string device;
    std::string command;
    std::vector<FieldMeta> fields;
    std::vector<Support> supports;
};

static void parseDeviceCommand(const std::string& name, std::string& device, std::string& command) {
    std::string n = name;
    std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return std::tolower(c); });
    
    static const std::vector<std::string> devices = {
        "dc_motor", "servo", "stepper", "laser", "led", "rotary_encoder", 
        "limit_switch", "diode", "ccd", "spectroscopy", "fluorometry",
        "power", "swerve", "arm", "kill", "pcb"
    };
    
    for (const auto& d : devices) {
        if (n.find(d) == 0) {
            device = d;
            command = n.substr(d.size());
            while(!command.empty() && (command[0] == '_' || command[0] == '-')) command.erase(0, 1);
            if (command.empty()) command = "status";
            
            if (command.size() > 6 && command.substr(command.size()-6) == "_pcb_c") command = command.substr(0, command.size()-6) + "_control";
            if (command.size() > 6 && command.substr(command.size()-6) == "_pcb_r") command = command.substr(0, command.size()-6) + "_response";
            return;
        }
    }

    // Fallback: search for device anywhere in the name
    for (const auto& d : devices) {
        size_t pos = n.find(d);
        if (pos != std::string::npos) {
            device = d;
            command = n;
            command.erase(pos, d.size());
            while(!command.empty() && (command[0] == '_' || command[0] == '-')) command.erase(0, 1);
            while(!command.empty() && (command.back() == '_' || command.back() == '-')) command.pop_back();
            if (command.empty()) command = "status";
            return;
        }
    }
    
    device = "System";
    command = n;
}

std::string generate(const CppCAN::CANDatabase& db, const std::string& dbc_filename, const std::string& dbc_hash, const std::string& date) {
    std::map<std::pair<std::string, std::string>, GroupMeta> api;
    std::set<std::string> pcbs;

    for (const auto& [id, frame] : db) {
        std::string frame_name = frame.name();
        std::string pcb = frame_name;
        size_t p = pcb.find("_PCB");
        if (p != std::string::npos) pcb = pcb.substr(0, p);
        pcbs.insert(pcb);

        int mux_sb = 0, mux_len = 0;
        bool has_mux = false;
        for (const auto& [sn, sig] : frame) {
            if (sig.mux_type() == CppCAN::CANSignal::Multiplexer) {
                mux_sb = sig.start_bit(); mux_len = sig.length(); has_mux = true; break;
            }
        }

        if (has_mux) {
            std::map<int, std::vector<std::pair<std::string, const CppCAN::CANSignal*>>> byMux;
            for (const auto& [sn, sig] : frame)
                if (sig.mux_type() == CppCAN::CANSignal::MuxedSignal) byMux[sig.mux_value()].push_back({sn, &sig});

            using LayoutKey = std::vector<std::tuple<std::string, int, int, std::string>>;
            std::map<LayoutKey, std::vector<int>> layouts;
            for (auto& [mv, sigs] : byMux) {
                LayoutKey key;
                for (auto& s : sigs) {
                    std::string stem; int port;
                    if (splitPortSuffix(s.first, stem, port))
                        key.push_back({stem, (int)s.second->start_bit(), (int)s.second->length(), cppType(*s.second)});
                    else
                        key.push_back({s.first, (int)s.second->start_bit(), (int)s.second->length(), cppType(*s.second)});
                }
                std::sort(key.begin(), key.end());
                layouts[key].push_back(mv);
            }

            for (auto& [key, mvs] : layouts) {
                if (key.empty()) continue;
                std::sort(mvs.begin(), mvs.end());
                std::vector<std::string> stems;
                for (auto& t : key) stems.push_back(std::get<0>(t));
                
                std::string dev, cmd; 
                parseDeviceCommand(stems[0], dev, cmd);
                
                auto& gm = api[{dev, cmd}];
                gm.device = dev; gm.command = cmd;
                if (gm.fields.empty()) {
                    std::string cmd_full = dev + "_" + cmd;
                    for (auto& [stem, sb, len, type] : key) {
                        std::string name = stem;
                        if (name == cmd_full) name = "value";
                        else if (name.find(cmd_full + "_") == 0) name = name.substr(cmd_full.size() + 1);
                        else if (name.find(dev + "_") == 0) name = name.substr(dev.size() + 1);
                        
                        if (name.empty()) name = "value";
                        gm.fields.push_back({stem, toIdentifier(name), (unsigned)sb, (unsigned)len, type});
                    }
                }
                int maxB = mux_sb + mux_len;
                for (auto& f : gm.fields) maxB = std::max(maxB, (int)(f.start_bit + f.length));
                gm.supports.push_back({pcb, (uint32_t)frame.can_id(), mvs[0], (int)mvs.size(), mux_sb, mux_len, (maxB + 7) / 8});
            }
        } else {
            std::string dev, cmd; 
            parseDeviceCommand(frame_name, dev, cmd);
            auto& gm = api[{dev, cmd}];
            gm.device = dev; gm.command = cmd;
            if (gm.fields.empty()) {
                std::string cmd_full = dev + "_" + cmd;
                for (const auto& [sn, sig] : frame) {
                    std::string name = sn;
                    if (name == cmd_full) name = "value";
                    else if (name.find(cmd_full + "_") == 0) name = name.substr(cmd_full.size() + 1);
                    else if (name.find(dev + "_") == 0) name = name.substr(dev.size() + 1);
                    
                    if (name.empty()) name = "value";
                    gm.fields.push_back({sn, toIdentifier(name), sig.start_bit(), sig.length(), cppType(sig)});
                }
            }
            gm.supports.push_back({pcb, (uint32_t)frame.can_id(), 0, 1, 0, 0, (int)frame.dlc()});
        }
    }

    std::ostringstream out;
    out << "// Generated from " << dbc_filename << " (Hash: " << dbc_hash << ") on " << date << "\n" << HEADER_BOILERPLATE;
    out << "enum class Subsystem {\n";
    for (const auto& n : pcbs) out << "    " << toCamelCase(n) << ",\n";
    out << "    Unknown\n};\n\n";

    std::map<std::string, std::vector<const GroupMeta*>> byDev;
    for (auto& [k, g] : api) byDev[g.device].push_back(&g);

    for (auto& [dev, gms] : byDev) {
        out << "namespace " << toCamelCase(dev) << " {\n\n";
        for (const auto* g : gms) {
            std::string cName = toCamelCase(g->command);
            out << "struct " << cName << " {\n";
            for (const auto& f : g->fields) out << "    " << f.type << " " << f.api_name << "{};\n";
            out << "};\n\n";

            out << "inline CanFrame encode_" << toIdentifier(g->command) << "(Subsystem sub, uint8_t port, const " << cName << "& data) {\n"
                << "    CanFrame f{};\n    switch(sub) {\n";
            std::set<std::string> seenPcb;
            for (const auto& s : g->supports) {
                if (seenPcb.count(s.pcb)) continue;
                seenPcb.insert(s.pcb);
                out << "        case Subsystem::" << toCamelCase(s.pcb) << ":\n"
                    << "            f.id = 0x" << std::hex << s.id << std::dec << "; f.dlc = " << s.dlc << ";\n";
                if (s.mux_len > 0) out << "            encode_field<" << s.mux_start << "," << s.mux_len << ">(f.data.data(), 0x" << std::hex << s.base_mux << std::dec << " + port);\n";
                out << "            break;\n";
            }
            out << "        default: break;\n    }\n";
            for (const auto& f : g->fields) out << "    encode_field<" << f.start_bit << "," << f.length << ">(f.data.data(), " << (f.type == "bool" ? "static_cast<uint8_t>(data." + f.api_name + ")" : "data." + f.api_name) << ");\n";
            out << "    return f;\n}\n\n";

            out << "struct " << cName << "Result { uint8_t port; " << cName << " data; };\n";
            out << "inline std::optional<" << cName << "Result> decode_" << toIdentifier(g->command) << "(const CanFrame& f) {\n";
            std::set<std::tuple<uint32_t, int, int>> seenIdMux;
            for (const auto& s : g->supports) {
                if (seenIdMux.count({s.id, s.base_mux, s.num_ports})) continue;
                seenIdMux.insert({s.id, s.base_mux, s.num_ports});
                out << "    if (f.id == 0x" << std::hex << s.id << std::dec;
                if (s.mux_len > 0) out << " && decode_field<" << s.mux_start << "," << s.mux_len << ",uint8_t>(f.data.data()) >= 0x" << std::hex << s.base_mux << std::dec << " && decode_field<" << s.mux_start << "," << s.mux_len << ",uint8_t>(f.data.data()) < 0x" << std::hex << (s.base_mux + s.num_ports) << std::dec;
                out << ") {\n        return " << cName << "Result{" << (s.mux_len > 0 ? "static_cast<uint8_t>(decode_field<" + std::to_string(s.mux_start) + "," + std::to_string(s.mux_len) + ",uint8_t>(f.data.data()) - 0x" + (std::ostringstream() << std::hex << s.base_mux).str() + ")" : "0") << ", " << cName << "{";
                for (size_t i = 0; i < g->fields.size(); ++i) out << "decode_field<" << g->fields[i].start_bit << "," << g->fields[i].length << "," << (g->fields[i].type == "bool" ? "uint8_t" : g->fields[i].type) << ">(f.data.data())" << (g->fields[i].type == "bool" ? "!=0" : "") << (i == g->fields.size() - 1 ? "" : ", ");
                out << "}};\n    }\n";
            }
            out << "    return std::nullopt;\n}\n\n";
        }
        out << "} // namespace " << toCamelCase(dev) << "\n\n";
    }
    out << "} // namespace Protocol\n";
    return out.str();
}
