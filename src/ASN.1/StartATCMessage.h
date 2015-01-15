/*
 * Generated by asn1c-0.9.26 (http://lionet.info/asn1c)
 * From ASN.1 module "VSNTestBed"
 * 	found in "Messages.asn1"
 * 	`asn1c -fnative-types`
 */

#ifndef	_StartATCMessage_H_
#define	_StartATCMessage_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "DetectorTypes.h"
#include <NativeReal.h>
#include "DescriptorTypes.h"
#include <BOOLEAN.h>
#include "CodingChoices.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* StartATCMessage */
typedef struct StartATCMessage {
	long	 framesPerSecond;
	DetectorTypes_t	 detectorType;
	double	 detectorThreshold;
	DescriptorTypes_t	 descriptorType;
	long	 descriptorLength;
	long	 maxNumberOfFeatures;
	BOOLEAN_t	 rotationInvariant;
	CodingChoices_t	 coding;
	BOOLEAN_t	 transferCoordinates;
	BOOLEAN_t	 transferScale;
	BOOLEAN_t	 transferOrientation;
	long	 numFeaturesPerBlock;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} StartATCMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_StartATCMessage;

#ifdef __cplusplus
}
#endif

#endif	/* _StartATCMessage_H_ */
#include <asn_internal.h>
