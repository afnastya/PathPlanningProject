#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = false;
    allowdiagonal = true;
    cutcorners = false;
    hweight = 1.0;
    searchtype = CN_SP_ST_DIJK;
}

EnvironmentOptions::EnvironmentOptions(bool AS, bool AD, bool CC, int MT, double HW, int ST)
{
    metrictype = MT;
    allowsqueeze = AS;
    allowdiagonal = AD;
    cutcorners = CC;
    hweight = HW;
    searchtype = ST;
}

