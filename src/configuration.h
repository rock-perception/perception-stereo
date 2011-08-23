#ifndef DENSE_STEREO_CONFIGURATION_H_
#define DENSE_STEREO_CONFIGURATION_H_

#include <libelas/elas.h>
#include "dense_stereo_types.h"

namespace stereo {

class Configuration {
public:
	Configuration();
	virtual ~Configuration();

	/** load precalculated default parameters */
	static void loadLibElasDefaultParameters(Elas::parameters &elasParam);
	
	/// load libelas configuration
	static void loadLibElasConfiguration(const libElasConfiguration &libElasParam, Elas::parameters &elasParam);
};
}

#endif /* DENSE_STEREO_CONFIGURATION_H_ */
