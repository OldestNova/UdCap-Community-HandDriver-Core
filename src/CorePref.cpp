//
// Created by max_3 on 25-6-20.
//

#include "CorePref.h"

void CorePref::setDefaultPrefPath(std::string path) {
    this->prefPath = path;
}

std::filesystem::path CorePref::getPrefPath() const {
    return this->prefPath;
}

CorePref::CorePref() {
    std::filesystem::path prefDir("prefs");
    this->prefPath = std::filesystem::relative(std::filesystem::current_path(), prefDir).string();
}

CorePref& CorePref::getInstance() {
    static CorePref instance;
    return instance;
}