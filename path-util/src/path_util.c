#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "path_util.h"

#ifndef BASE_PATH
#error "BASE_PATH is not defined by compiler"
#endif

const char *
getBasePath()
{
    return BASE_PATH;
}

const char *
getDataPath()
{
//    return BASE_PATH "/data";
    return BASE_PATH "/../src/director_drs/data";
}

const char *
getConfigPath()
{
//    return BASE_PATH "/config";
    return BASE_PATH "/../src/director_drs/config";
}

const char *
getModelsPath()
{
//    return BASE_PATH "/models";
    return BASE_PATH "/../src/director_drs/models";
}
