#include "dense_stereo_types.h"
#include <libelas/elas.h>
#include "configuration.h"

using namespace stereo;

libElasConfiguration::libElasConfiguration()
{
    // copy default parameters from libelas
    Elas::parameters params;
    copyFromElas( &params, this );
}

