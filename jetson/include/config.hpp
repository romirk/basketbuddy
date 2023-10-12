#pragma once

#include <vector>
#include <string>
#include <boost/program_options/variables_map.hpp>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/program_options/variables_map.hpp>

class Config {
public:

    static boost::program_options::variables_map *load(const char *path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            printf("Failed to open config file %s\n", path);
            exit(1);
        }
        printf("Loading config from %s\n", path);

        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(file, pt);

        // Convert to variables_map
        auto *vm = new boost::program_options::variables_map();
        for (auto &section: pt) {
            for (auto &key: section.second) {
                printf("  %s.%s = %s\n", section.first.c_str(), key.first.c_str(), key.second.data().c_str());
                vm->insert(std::make_pair(section.first + "." + key.first,
                                          boost::program_options::variable_value(key.second.data(), false)));
            }
        }

        printf("done configuring robot %s.\n", vm->at("Options.name").as<std::string>().c_str());
        return vm;
    }

};