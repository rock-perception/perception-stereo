/*
 * configuration.h
 *
 *  Created on: Apr 13, 2011
 *      Author: jfraedrich
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <opencv/cv.h>
#include "calibrationparameters.h"
#include "elas.h"

class Configuration {
public:
	Configuration();
	virtual ~Configuration();

	/** load precalculated default parameters */
	static void loadDefaultParameters(CalibrationParameters &calParam, Elas::parameters &elasParam);
	/// load precalculated parameters from file
	static void loadConfigurationFromFile(const std::string &filename, CalibrationParameters &calParam, Elas::parameters &elasParam);
	/// save parameters to file
	static void saveConfigurationFile(const std::string &filename, CalibrationParameters &calParam, Elas::parameters &elasParam);
};

#endif /* CONFIGURATION_H_ */
