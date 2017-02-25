#
# OMNeT++/OMNEST Makefile for CooperativePositioning
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out -I../veins/src -I../veins/src/veins/modules/mobility/traci -L../veins/out/$$\(CONFIGNAME\)/src -lproj -lveins -KVEINS_PROJ=../veins
#

# Name of target to be created (-o option)
TARGET = CooperativePositioning$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I../veins/src \
    -I../veins/src/veins/modules/mobility/traci \
    -I. \
    -ITypes \
    -Icommunication \
    -Ilocalization \
    -Ilocalization/DeadReckoning \
    -Ilocalization/Filters \
    -Ilocalization/GPS \
    -Ilocalization/GPS/mean_std_dev \
    -Ilocalization/GPS/outages \
    -Ilocalization/GPS/outagesxy \
    -Ilocalization/GeographicLib \
    -Ilocalization/GeographicLib/doc \
    -Ilocalization/GeographicLib/include \
    -Ilocalization/GeographicLib/include/GeographicLib \
    -Ilocalization/GeographicLib/src \
    -Ilocalization/Graphs \
    -Ilocalization/Graphs/out \
    -Ilocalization/MapMatching \
    -Ilocalization/Multilateration \
    -Ilocalization/Outage \
    -Ilocalization/Projections \
    -Ilocalization/Projections/parameters \
    -Ilocalization/RSSI \
    -Ilocalization/jama_tnt \
    -Isimulations \
    -Isimulations/results \
    -Isimulations/results/results-old \
    -Isumoscenarios

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../veins/out/$(CONFIGNAME)/src  -lproj -lveins
LIBS += -Wl,-rpath,`abspath ../veins/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/Types/Types.o \
    $O/communication/LocAppCom.o \
    $O/localization/DeadReckoning/DeadReckoning.o \
    $O/localization/Filters/Filters.o \
    $O/localization/GPS/GPS.o \
    $O/localization/GeographicLib/src/GeodesicLineExact.o \
    $O/localization/GeographicLib/src/CircularEngine.o \
    $O/localization/GeographicLib/src/DMS.o \
    $O/localization/GeographicLib/src/Gnomonic.o \
    $O/localization/GeographicLib/src/Ellipsoid.o \
    $O/localization/GeographicLib/src/Utility.o \
    $O/localization/GeographicLib/src/Geohash.o \
    $O/localization/GeographicLib/src/GeodesicExact.o \
    $O/localization/GeographicLib/src/GeodesicExactC4.o \
    $O/localization/GeographicLib/src/EllipticFunction.o \
    $O/localization/GeographicLib/src/GeodesicLine.o \
    $O/localization/GeographicLib/src/MagneticModel.o \
    $O/localization/GeographicLib/src/SphericalEngine.o \
    $O/localization/GeographicLib/src/LocalCartesian.o \
    $O/localization/GeographicLib/src/GravityModel.o \
    $O/localization/GeographicLib/src/TransverseMercator.o \
    $O/localization/GeographicLib/src/NormalGravity.o \
    $O/localization/GeographicLib/src/GravityCircle.o \
    $O/localization/GeographicLib/src/Rhumb.o \
    $O/localization/GeographicLib/src/AzimuthalEquidistant.o \
    $O/localization/GeographicLib/src/MGRS.o \
    $O/localization/GeographicLib/src/GeoCoords.o \
    $O/localization/GeographicLib/src/LambertConformalConic.o \
    $O/localization/GeographicLib/src/PolarStereographic.o \
    $O/localization/GeographicLib/src/MagneticCircle.o \
    $O/localization/GeographicLib/src/Geocentric.o \
    $O/localization/GeographicLib/src/CassiniSoldner.o \
    $O/localization/GeographicLib/src/OSGB.o \
    $O/localization/GeographicLib/src/PolygonArea.o \
    $O/localization/GeographicLib/src/Geoid.o \
    $O/localization/GeographicLib/src/AlbersEqualArea.o \
    $O/localization/GeographicLib/src/TransverseMercatorExact.o \
    $O/localization/GeographicLib/src/UTMUPS.o \
    $O/localization/GeographicLib/src/Math.o \
    $O/localization/GeographicLib/src/Geodesic.o \
    $O/localization/GeographicLib/src/GARS.o \
    $O/localization/GeographicLib/src/Accumulator.o \
    $O/localization/GeographicLib/src/Georef.o \
    $O/localization/MapMatching/MapMatching.o \
    $O/localization/Multilateration/Multilateration.o \
    $O/localization/Outage/Outage.o \
    $O/localization/Projections/Projection.o \
    $O/localization/RSSI/FreeSpaceModel.o \
    $O/localization/RSSI/TwoRayInterferenceModel.o

# Message files
MSGFILES =

# SM files
SMFILES =

# Other makefile variables (-K)
VEINS_PROJ=../veins

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f CooperativePositioning CooperativePositioning.exe libCooperativePositioning.so libCooperativePositioning.a libCooperativePositioning.dll libCooperativePositioning.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f Types/*_m.cc Types/*_m.h Types/*_sm.cc Types/*_sm.h
	$(Q)-rm -f communication/*_m.cc communication/*_m.h communication/*_sm.cc communication/*_sm.h
	$(Q)-rm -f localization/*_m.cc localization/*_m.h localization/*_sm.cc localization/*_sm.h
	$(Q)-rm -f localization/DeadReckoning/*_m.cc localization/DeadReckoning/*_m.h localization/DeadReckoning/*_sm.cc localization/DeadReckoning/*_sm.h
	$(Q)-rm -f localization/Filters/*_m.cc localization/Filters/*_m.h localization/Filters/*_sm.cc localization/Filters/*_sm.h
	$(Q)-rm -f localization/GPS/*_m.cc localization/GPS/*_m.h localization/GPS/*_sm.cc localization/GPS/*_sm.h
	$(Q)-rm -f localization/GPS/mean_std_dev/*_m.cc localization/GPS/mean_std_dev/*_m.h localization/GPS/mean_std_dev/*_sm.cc localization/GPS/mean_std_dev/*_sm.h
	$(Q)-rm -f localization/GPS/outages/*_m.cc localization/GPS/outages/*_m.h localization/GPS/outages/*_sm.cc localization/GPS/outages/*_sm.h
	$(Q)-rm -f localization/GPS/outagesxy/*_m.cc localization/GPS/outagesxy/*_m.h localization/GPS/outagesxy/*_sm.cc localization/GPS/outagesxy/*_sm.h
	$(Q)-rm -f localization/GeographicLib/*_m.cc localization/GeographicLib/*_m.h localization/GeographicLib/*_sm.cc localization/GeographicLib/*_sm.h
	$(Q)-rm -f localization/GeographicLib/doc/*_m.cc localization/GeographicLib/doc/*_m.h localization/GeographicLib/doc/*_sm.cc localization/GeographicLib/doc/*_sm.h
	$(Q)-rm -f localization/GeographicLib/include/*_m.cc localization/GeographicLib/include/*_m.h localization/GeographicLib/include/*_sm.cc localization/GeographicLib/include/*_sm.h
	$(Q)-rm -f localization/GeographicLib/include/GeographicLib/*_m.cc localization/GeographicLib/include/GeographicLib/*_m.h localization/GeographicLib/include/GeographicLib/*_sm.cc localization/GeographicLib/include/GeographicLib/*_sm.h
	$(Q)-rm -f localization/GeographicLib/src/*_m.cc localization/GeographicLib/src/*_m.h localization/GeographicLib/src/*_sm.cc localization/GeographicLib/src/*_sm.h
	$(Q)-rm -f localization/Graphs/*_m.cc localization/Graphs/*_m.h localization/Graphs/*_sm.cc localization/Graphs/*_sm.h
	$(Q)-rm -f localization/Graphs/out/*_m.cc localization/Graphs/out/*_m.h localization/Graphs/out/*_sm.cc localization/Graphs/out/*_sm.h
	$(Q)-rm -f localization/MapMatching/*_m.cc localization/MapMatching/*_m.h localization/MapMatching/*_sm.cc localization/MapMatching/*_sm.h
	$(Q)-rm -f localization/Multilateration/*_m.cc localization/Multilateration/*_m.h localization/Multilateration/*_sm.cc localization/Multilateration/*_sm.h
	$(Q)-rm -f localization/Outage/*_m.cc localization/Outage/*_m.h localization/Outage/*_sm.cc localization/Outage/*_sm.h
	$(Q)-rm -f localization/Projections/*_m.cc localization/Projections/*_m.h localization/Projections/*_sm.cc localization/Projections/*_sm.h
	$(Q)-rm -f localization/Projections/parameters/*_m.cc localization/Projections/parameters/*_m.h localization/Projections/parameters/*_sm.cc localization/Projections/parameters/*_sm.h
	$(Q)-rm -f localization/RSSI/*_m.cc localization/RSSI/*_m.h localization/RSSI/*_sm.cc localization/RSSI/*_sm.h
	$(Q)-rm -f localization/jama_tnt/*_m.cc localization/jama_tnt/*_m.h localization/jama_tnt/*_sm.cc localization/jama_tnt/*_sm.h
	$(Q)-rm -f simulations/*_m.cc simulations/*_m.h simulations/*_sm.cc simulations/*_sm.h
	$(Q)-rm -f simulations/results/*_m.cc simulations/results/*_m.h simulations/results/*_sm.cc simulations/results/*_sm.h
	$(Q)-rm -f simulations/results/results-old/*_m.cc simulations/results/results-old/*_m.h simulations/results/results-old/*_sm.cc simulations/results/results-old/*_sm.h
	$(Q)-rm -f sumoscenarios/*_m.cc sumoscenarios/*_m.h sumoscenarios/*_sm.cc sumoscenarios/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc Types/*.cc communication/*.cc localization/*.cc localization/DeadReckoning/*.cc localization/Filters/*.cc localization/GPS/*.cc localization/GPS/mean_std_dev/*.cc localization/GPS/outages/*.cc localization/GPS/outagesxy/*.cc localization/GeographicLib/*.cc localization/GeographicLib/doc/*.cc localization/GeographicLib/include/*.cc localization/GeographicLib/include/GeographicLib/*.cc localization/GeographicLib/src/*.cc localization/Graphs/*.cc localization/Graphs/out/*.cc localization/MapMatching/*.cc localization/Multilateration/*.cc localization/Outage/*.cc localization/Projections/*.cc localization/Projections/parameters/*.cc localization/RSSI/*.cc localization/jama_tnt/*.cc simulations/*.cc simulations/results/*.cc simulations/results/results-old/*.cc sumoscenarios/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/Types/Types.o: Types/Types.cc \
	Types/Types.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/communication/LocAppCom.o: communication/LocAppCom.cc \
	Types/Types.h \
	communication/LocAppCom.h \
	localization/DeadReckoning/DeadReckoning.h \
	localization/Filters/Filters.h \
	localization/GPS/GPS.h \
	localization/MapMatching/MapMatching.h \
	localization/Multilateration/Multilateration.h \
	localization/Outage/Outage.h \
	localization/Projections/Projection.h \
	localization/RSSI/FreeSpaceModel.h \
	localization/RSSI/TwoRayInterferenceModel.h \
	localization/jama_tnt/jama_qr.h \
	localization/jama_tnt/tnt.h \
	localization/jama_tnt/tnt_array1d.h \
	localization/jama_tnt/tnt_array1d_utils.h \
	localization/jama_tnt/tnt_array2d.h \
	localization/jama_tnt/tnt_array2d_utils.h \
	localization/jama_tnt/tnt_array3d.h \
	localization/jama_tnt/tnt_array3d_utils.h \
	localization/jama_tnt/tnt_cmat.h \
	localization/jama_tnt/tnt_fortran_array1d.h \
	localization/jama_tnt/tnt_fortran_array1d_utils.h \
	localization/jama_tnt/tnt_fortran_array2d.h \
	localization/jama_tnt/tnt_fortran_array2d_utils.h \
	localization/jama_tnt/tnt_fortran_array3d.h \
	localization/jama_tnt/tnt_fortran_array3d_utils.h \
	localization/jama_tnt/tnt_i_refvec.h \
	localization/jama_tnt/tnt_math_utils.h \
	localization/jama_tnt/tnt_sparse_matrix_csr.h \
	localization/jama_tnt/tnt_stopwatch.h \
	localization/jama_tnt/tnt_subscript.h \
	localization/jama_tnt/tnt_vec.h \
	localization/jama_tnt/tnt_version.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/BaseConnectionManager.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/ChannelAccess.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/NicEntry.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseApplLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseBattery.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseMobility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseModule.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseWorldUtility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BatteryAccess.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/FindModule.h \
	$(VEINS_PROJ)/src/veins/base/utils/HostState.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/Move.h \
	$(VEINS_PROJ)/src/veins/base/utils/NetwToMacControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/utils/PassedMessage.h \
	$(VEINS_PROJ)/src/veins/base/utils/SimpleAddress.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_PROJ)/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	$(VEINS_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIBuffer.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIColor.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICommandInterface.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIConnection.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICoord.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIMobility.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIScenarioManager.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/Obstacle.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/ObstacleControl.h \
	$(VEINS_PROJ)/src/veins/modules/utility/Consts80211p.h \
	$(VEINS_PROJ)/src/veins/modules/world/annotations/AnnotationManager.h
$O/localization/DeadReckoning/DeadReckoning.o: localization/DeadReckoning/DeadReckoning.cc \
	Types/Types.h \
	localization/DeadReckoning/DeadReckoning.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/Filters/Filters.o: localization/Filters/Filters.cc \
	localization/Filters/Filters.h
$O/localization/GPS/GPS.o: localization/GPS/GPS.cc \
	Types/Types.h \
	localization/GPS/GPS.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/GeographicLib/src/Accumulator.o: localization/GeographicLib/src/Accumulator.cc
$O/localization/GeographicLib/src/AlbersEqualArea.o: localization/GeographicLib/src/AlbersEqualArea.cc
$O/localization/GeographicLib/src/AzimuthalEquidistant.o: localization/GeographicLib/src/AzimuthalEquidistant.cc
$O/localization/GeographicLib/src/CassiniSoldner.o: localization/GeographicLib/src/CassiniSoldner.cc
$O/localization/GeographicLib/src/CircularEngine.o: localization/GeographicLib/src/CircularEngine.cc
$O/localization/GeographicLib/src/DMS.o: localization/GeographicLib/src/DMS.cc
$O/localization/GeographicLib/src/Ellipsoid.o: localization/GeographicLib/src/Ellipsoid.cc
$O/localization/GeographicLib/src/EllipticFunction.o: localization/GeographicLib/src/EllipticFunction.cc
$O/localization/GeographicLib/src/GARS.o: localization/GeographicLib/src/GARS.cc
$O/localization/GeographicLib/src/GeoCoords.o: localization/GeographicLib/src/GeoCoords.cc
$O/localization/GeographicLib/src/Geocentric.o: localization/GeographicLib/src/Geocentric.cc
$O/localization/GeographicLib/src/Geodesic.o: localization/GeographicLib/src/Geodesic.cc
$O/localization/GeographicLib/src/GeodesicExact.o: localization/GeographicLib/src/GeodesicExact.cc
$O/localization/GeographicLib/src/GeodesicExactC4.o: localization/GeographicLib/src/GeodesicExactC4.cc
$O/localization/GeographicLib/src/GeodesicLine.o: localization/GeographicLib/src/GeodesicLine.cc
$O/localization/GeographicLib/src/GeodesicLineExact.o: localization/GeographicLib/src/GeodesicLineExact.cc
$O/localization/GeographicLib/src/Geohash.o: localization/GeographicLib/src/Geohash.cc
$O/localization/GeographicLib/src/Geoid.o: localization/GeographicLib/src/Geoid.cc
$O/localization/GeographicLib/src/Georef.o: localization/GeographicLib/src/Georef.cc
$O/localization/GeographicLib/src/Gnomonic.o: localization/GeographicLib/src/Gnomonic.cc
$O/localization/GeographicLib/src/GravityCircle.o: localization/GeographicLib/src/GravityCircle.cc
$O/localization/GeographicLib/src/GravityModel.o: localization/GeographicLib/src/GravityModel.cc
$O/localization/GeographicLib/src/LambertConformalConic.o: localization/GeographicLib/src/LambertConformalConic.cc
$O/localization/GeographicLib/src/LocalCartesian.o: localization/GeographicLib/src/LocalCartesian.cc
$O/localization/GeographicLib/src/MGRS.o: localization/GeographicLib/src/MGRS.cc
$O/localization/GeographicLib/src/MagneticCircle.o: localization/GeographicLib/src/MagneticCircle.cc
$O/localization/GeographicLib/src/MagneticModel.o: localization/GeographicLib/src/MagneticModel.cc
$O/localization/GeographicLib/src/Math.o: localization/GeographicLib/src/Math.cc
$O/localization/GeographicLib/src/NormalGravity.o: localization/GeographicLib/src/NormalGravity.cc
$O/localization/GeographicLib/src/OSGB.o: localization/GeographicLib/src/OSGB.cc
$O/localization/GeographicLib/src/PolarStereographic.o: localization/GeographicLib/src/PolarStereographic.cc
$O/localization/GeographicLib/src/PolygonArea.o: localization/GeographicLib/src/PolygonArea.cc
$O/localization/GeographicLib/src/Rhumb.o: localization/GeographicLib/src/Rhumb.cc
$O/localization/GeographicLib/src/SphericalEngine.o: localization/GeographicLib/src/SphericalEngine.cc
$O/localization/GeographicLib/src/TransverseMercator.o: localization/GeographicLib/src/TransverseMercator.cc
$O/localization/GeographicLib/src/TransverseMercatorExact.o: localization/GeographicLib/src/TransverseMercatorExact.cc
$O/localization/GeographicLib/src/UTMUPS.o: localization/GeographicLib/src/UTMUPS.cc
$O/localization/GeographicLib/src/Utility.o: localization/GeographicLib/src/Utility.cc
$O/localization/MapMatching/MapMatching.o: localization/MapMatching/MapMatching.cc \
	Types/Types.h \
	localization/MapMatching/MapMatching.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/Multilateration/Multilateration.o: localization/Multilateration/Multilateration.cc \
	Types/Types.h \
	localization/Multilateration/Multilateration.h \
	localization/jama_tnt/jama_qr.h \
	localization/jama_tnt/tnt.h \
	localization/jama_tnt/tnt_array1d.h \
	localization/jama_tnt/tnt_array1d_utils.h \
	localization/jama_tnt/tnt_array2d.h \
	localization/jama_tnt/tnt_array2d_utils.h \
	localization/jama_tnt/tnt_array3d.h \
	localization/jama_tnt/tnt_array3d_utils.h \
	localization/jama_tnt/tnt_cmat.h \
	localization/jama_tnt/tnt_fortran_array1d.h \
	localization/jama_tnt/tnt_fortran_array1d_utils.h \
	localization/jama_tnt/tnt_fortran_array2d.h \
	localization/jama_tnt/tnt_fortran_array2d_utils.h \
	localization/jama_tnt/tnt_fortran_array3d.h \
	localization/jama_tnt/tnt_fortran_array3d_utils.h \
	localization/jama_tnt/tnt_i_refvec.h \
	localization/jama_tnt/tnt_math_utils.h \
	localization/jama_tnt/tnt_sparse_matrix_csr.h \
	localization/jama_tnt/tnt_stopwatch.h \
	localization/jama_tnt/tnt_subscript.h \
	localization/jama_tnt/tnt_vec.h \
	localization/jama_tnt/tnt_version.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/Outage/Outage.o: localization/Outage/Outage.cc \
	Types/Types.h \
	localization/Outage/Outage.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/Projections/Projection.o: localization/Projections/Projection.cc \
	Types/Types.h \
	localization/Projections/Projection.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/localization/RSSI/FreeSpaceModel.o: localization/RSSI/FreeSpaceModel.cc \
	localization/RSSI/FreeSpaceModel.h
$O/localization/RSSI/TwoRayInterferenceModel.o: localization/RSSI/TwoRayInterferenceModel.cc \
	localization/RSSI/TwoRayInterferenceModel.h

