#ifndef DENSE_STEREO_CONFIGURATION_H_
#define DENSE_STEREO_CONFIGURATION_H_

#include <libelas/elas.h>
#include "dense_stereo_types.h"

namespace stereo {
    void copyFromElas( const Elas::parameters *params, libElasConfiguration *config );
    void copyToElas( const libElasConfiguration *config, Elas::parameters *params );
}

#endif /* DENSE_STEREO_CONFIGURATION_H_ */
