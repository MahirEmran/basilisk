%module activeGuidance

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "activeGuidance.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "activeGuidance.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
struct AttRefMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
