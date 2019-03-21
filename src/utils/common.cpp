#include "utils/common.h"

string Common::dataPack(const string &name, const string &content){
    Data buff;
    buff.name = name;
    buff.content = content;
    buff.total_len.resize(8);
    buff.name_len = to_string(buff.name.size());
    buff.total_len = to_string(buff.start.size() + buff.name_len.size() + buff.total_len.size() + buff.name.size() + buff.content.size() + buff.terminator.size());
    buff.total_len.resize(8);
    buff.all = buff.start + buff.name_len + buff.total_len + buff.name + buff.content + buff.terminator;
    return buff.all;
}