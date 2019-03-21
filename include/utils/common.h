#pragma once
#include <iostream>
#include <string>

using namespace std;

class Common{
public:
    string dataPack(const string &name, const string &content);

private:
    struct Data
    {
        string start{"$START"};
        string name_len;
        string total_len;
        string name;
        string content;
        string terminator{"$END"};
        string all;
    };
};

