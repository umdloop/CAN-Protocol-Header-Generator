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

namespace Protocol {

template<int StartBit, int Length, typename UInt>
inline void encode_field(uint8_t* data, UInt value) {
    using U64 = uint64_t;
    using Uns = std::make_unsigned_t<UInt>;
    U64 word;
    std::memcpy(&word, data, 8);
    constexpr U64 mask = (Length == 64) ? ~U64{0} : ((U64{1} << Length) - 1);
    word = (word & ~(mask << StartBit)) | ((static_cast<U64>(static_cast<Uns>(value)) & mask) << StartBit);
    std::memcpy(data, &word, 8);
}

template<int StartBit, int Length, typename T = uint64_t>
inline T decode_field(const uint8_t* data) {
    using U64 = uint64_t;
    U64 word;
    std::memcpy(&word, data, 8);
    constexpr U64 mask = (Length == 64) ? ~U64{0} : ((U64{1} << Length) - 1);
    U64 raw = (word >> StartBit) & mask;
    if constexpr (std::is_signed_v<T> && Length < 64) {
        if (raw >> (Length - 1))
            raw |= ~mask;
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

template<typename T> T decode(const CanFrame& frame);

)";

struct GroupField {
    std::string stem;
    unsigned int start_bit;
    unsigned int length;
    std::string cpp_type;
};

struct MuxCommandGroup {
    std::string group_stem;
    int base_mux;
    int num_ports;
    unsigned int mux_start_bit;
    unsigned int mux_length;
    std::vector<GroupField> fields;
    unsigned int dlc;
};

static std::string commonPrefix(const std::vector<std::string>& stems) {
    if (stems.empty()) return "";
    std::string pfx = stems[0];
    for (const auto& s : stems) {
        size_t i = 0;
        while (i < pfx.size() && i < s.size() && pfx[i] == s[i]) ++i;
        pfx = pfx.substr(0, i);
    }
    auto pos = pfx.rfind('_');
    if (pos != std::string::npos) pfx = pfx.substr(0, pos);
    while (!pfx.empty() && pfx.back() == '_') pfx.pop_back();
    return pfx;
}

static std::string commonSuffix(const std::vector<std::string>& stems) {
    if (stems.empty()) return "";
    auto rev = [](std::string s){ std::reverse(s.begin(), s.end()); return s; };
    std::vector<std::string> reversed;
    for (const auto& s : stems) reversed.push_back(rev(s));
    std::string pfx = reversed[0];
    for (const auto& s : reversed) {
        size_t i = 0;
        while (i < pfx.size() && i < s.size() && pfx[i] == s[i]) ++i;
        pfx = pfx.substr(0, i);
    }
    std::reverse(pfx.begin(), pfx.end());
    while (!pfx.empty() && pfx.front() == '_') pfx.erase(pfx.begin());
    return pfx;
}

static std::string groupStem(const std::vector<GroupField>& fields, int base_mux) {
    if (fields.size() == 1) return fields[0].stem;
    std::vector<std::string> stems;
    for (const auto& f : fields) stems.push_back(f.stem);
    std::string s = commonSuffix(stems);
    if (s.size() >= 4) return s;
    s = commonPrefix(stems);
    if (s.size() >= 4) return s;
    std::ostringstream oss;
    oss << "cmd_0x" << std::hex << base_mux;
    return oss.str();
}

static std::vector<MuxCommandGroup> findMuxGroups(const CppCAN::CANFrame& frame) {
    unsigned int mux_start = 0, mux_len = 8;
    for (const auto& [sn, sig] : frame) {
        if (sig.mux_type() == CppCAN::CANSignal::Multiplexer) {
            mux_start = sig.start_bit();
            mux_len   = sig.length();
            break;
        }
    }

    struct SigInfo { std::string stem; int port; unsigned int start_bit; unsigned int length; std::string type; };
    std::map<int, std::vector<SigInfo>> byMuxVal;
    for (const auto& [sn, sig] : frame) {
        if (sig.mux_type() != CppCAN::CANSignal::MuxedSignal) continue;
        std::string stem; int port;
        if (splitPortSuffix(sig.name(), stem, port))
            byMuxVal[sig.mux_value()].push_back({stem, port, sig.start_bit(), sig.length(), cppType(sig)});
    }
    if (byMuxVal.empty()) return {};

    using LayoutKey = std::vector<std::tuple<std::string, unsigned int, unsigned int, std::string>>;
    std::map<LayoutKey, std::vector<int>> layoutToMuxVals;
    for (auto& [mv, sigs] : byMuxVal) {
        LayoutKey key;
        for (auto& si : sigs) key.push_back({si.stem, si.start_bit, si.length, si.type});
        std::sort(key.begin(), key.end());
        layoutToMuxVals[key].push_back(mv);
    }

    std::vector<MuxCommandGroup> groups;
    for (auto& [key, muxVals] : layoutToMuxVals) {
        std::sort(muxVals.begin(), muxVals.end());
        bool consecutive = true;
        for (int i = 1; i < (int)muxVals.size(); i++)
            if (muxVals[i] != muxVals[i-1] + 1) { consecutive = false; break; }
        if (!consecutive) continue;

        int base = muxVals[0];
        std::vector<GroupField> fields;
        for (auto& [stem, sb, len, type] : key) fields.push_back({stem, sb, len, type});
        std::sort(fields.begin(), fields.end(), [](const auto& a, const auto& b){ return a.start_bit < b.start_bit; });

        unsigned int maxBit = mux_start + mux_len - 1;
        for (auto& f : fields) maxBit = std::max(maxBit, f.start_bit + f.length - 1);

        groups.push_back({toIdentifier(groupStem(fields, base)), base, (int)muxVals.size(), mux_start, mux_len, std::move(fields), (maxBit / 8) + 1});
    }

    std::sort(groups.begin(), groups.end(), [](const auto& a, const auto& b){ return a.base_mux < b.base_mux; });
    std::map<std::string, int> stemCount;
    for (auto& g : groups) stemCount[g.group_stem]++;
    for (auto& g : groups) if (stemCount[g.group_stem] > 1) g.group_stem += "_0x" + (std::ostringstream() << std::hex << g.base_mux).str();
    return groups;
}

static std::string render(std::string tmpl, const std::map<std::string, std::string>& vars) {
    for (auto const& [key, val] : vars) {
        std::string placeholder = "{{" + key + "}}";
        size_t pos = 0;
        while ((pos = tmpl.find(placeholder, pos)) != std::string::npos) {
            tmpl.replace(pos, placeholder.length(), val);
            pos += val.length();
        }
    }
    return tmpl;
}

std::string generate(const CppCAN::CANDatabase& db, const std::string& dbc_filename, const std::string& dbc_hash, const std::string& date) {
    std::ostringstream out;
    out << "// Generated from " << dbc_filename << " (Hash: " << dbc_hash << ") on " << date << "\n" << HEADER_BOILERPLATE;

    struct Meta { uint32_t id; std::string name; };
    std::vector<Meta> metas;

    for (const auto& [key, frame] : db) {
        std::string name = toIdentifier(frame.name());
        metas.push_back({static_cast<uint32_t>(frame.can_id()), name});
        out << "struct " << name << " {\n" << "    static constexpr uint32_t ID  = 0x" << std::hex << frame.can_id() << std::dec << ";\n"
            << "    static constexpr uint8_t  DLC = " << (int)frame.dlc() << ";\n";
        for (const auto& [sn, sig] : frame)
            if (sig.mux_type() != CppCAN::CANSignal::MuxedSignal) out << "    " << cppType(sig) << " " << toIdentifier(sig.name()) << "{};\n";
        out << "};\n\n";
    }

    out << "enum class MessageType { ";
    for (const auto& m : metas) out << m.name << ", ";
    out << "Unknown };\n\ninline MessageType getMessageType(const CanFrame& frame) {\n    switch (frame.id) {\n";
    for (const auto& m : metas) out << "        case 0x" << std::hex << m.id << std::dec << ": return MessageType::" << m.name << ";\n";
    out << "        default: return MessageType::Unknown;\n    }\n}\n\n";

    for (const auto& [key, frame] : db) {
        std::string name = toIdentifier(frame.name()), eb, dbb;
        for (const auto& [sn, sig] : frame) {
            if (sig.mux_type() == CppCAN::CANSignal::MuxedSignal) continue;
            std::string f = toIdentifier(sig.name()), t = cppType(sig), sb = std::to_string(sig.start_bit()), l = std::to_string(sig.length());
            eb += "    encode_field<" + sb + ", " + l + ">(frame.data.data(), " + (t == "bool" ? "static_cast<uint8_t>(msg." + f + ")" : "msg." + f) + ");\n";
            dbb += "    msg." + f + " = decode_field<" + sb + ", " + l + ", " + (t == "bool" ? "uint8_t" : t) + ">(frame.data.data())" + (t == "bool" ? " != 0" : "") + ";\n";
        }
        out << render(R"(inline CanFrame encode(const {{n}}& msg) {
    CanFrame frame; frame.id = {{n}}::ID; frame.dlc = {{n}}::DLC;
{{eb}}    return frame;
}
template<> inline {{n}} decode<{{n}}>(const CanFrame& frame) {
    {{n}} msg;
{{db}}    return msg;
}
)", {{"n", name}, {"eb", eb}, {"db", dbb}});

        for (const auto& grp : findMuxGroups(frame)) {
            std::string sname = name + "_" + grp.group_stem + "_t", fdef, ef, df;
            for (const auto& f : grp.fields) {
                std::string t = f.cpp_type, sb = std::to_string(f.start_bit), l = std::to_string(f.length);
                fdef += "    " + t + " " + f.stem + "{};\n";
                ef += "    encode_field<" + sb + ", " + l + ">(frame.data.data(), " + (t == "bool" ? "static_cast<uint8_t>(data." + f.stem + ")" : "data." + f.stem) + ");\n";
                df += "    result." + f.stem + " = decode_field<" + sb + ", " + l + ", " + (t == "bool" ? "uint8_t" : t) + ">(frame.data.data())" + (t == "bool" ? " != 0" : "") + ";\n";
            }
            out << render(R"(struct {{sn}} {
    uint8_t port_id{};
{{fdef}}};
inline CanFrame encode(const {{sn}}& data) {
    CanFrame frame; frame.id = {{n}}::ID; frame.dlc = {{dlc}};
    encode_field<{{msb}}, {{ml}}>(frame.data.data(), static_cast<uint8_t>(0x{{bm}} + data.port_id));
{{ef}}    return frame;
}
template<> inline {{sn}} decode<{{sn}}>(const CanFrame& frame) {
    {{sn}} result; result.port_id = decode_field<{{msb}}, {{ml}}, uint8_t>(frame.data.data()) - 0x{{bm}};
{{df}}    return result;
}
)", {{"sn", sname}, {"fdef", fdef}, {"n", name}, {"dlc", std::to_string(grp.dlc)}, {"msb", std::to_string(grp.mux_start_bit)}, {"ml", std::to_string(grp.mux_length)}, {"bm", (std::ostringstream() << std::hex << grp.base_mux).str()}, {"ef", ef}, {"df", df}});
        }
    }
    out << "} // namespace Protocol\n";
    return out.str();
}
