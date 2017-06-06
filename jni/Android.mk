LOCAL_PATH :=$(call my-dir)

PCL_INCLUDE := G:/pcl-android/pcl-android
BOOST_ANDROID_INCLUDE := G:/pcl-android/boost-android
FLANN_INCLUDE := G:/pcl-android/flann-android
EIGEN_INCLUDE := G:/pcl-android/eigen

PCL_STATIC_LIB_DIR := $(PCL_INCLUDE)/lib
BOOST_STATIC_LIB_DIR := $(BOOST_ANDROID_INCLUDE)/lib
FLANN_STATIC_LIB_DIR := $(FLANN_INCLUDE)/lib

PCL_STATIC_LIBRARIES :=     pcl_common pcl_kdtree pcl_octree pcl_sample_consensus pcl_surface \
                            pcl_features pcl_keypoints pcl_search pcl_tracking pcl_filters pcl_ml \
                            pcl_registration pcl_segmentation pcl_io pcl_io_ply pcl_recognition
BOOST_STATIC_LIBRARIES :=   boost_date_time boost_iostreams boost_regex boost_system \
                            boost_filesystem boost_program_options boost_signals boost_thread
FLANN_STATIC_LIBRARIES :=   flann_s flann_cpp_s


define build_pcl_static
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(PCL_STATIC_LIB_DIR)/lib$1.a
    include $(PREBUILT_STATIC_LIBRARY)
endef

define build_boost_static
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(BOOST_STATIC_LIB_DIR)/lib$1.a
    include $(PREBUILT_STATIC_LIBRARY)
endef

define build_flann_static
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(FLANN_STATIC_LIB_DIR)/lib$1.a
    include $(PREBUILT_STATIC_LIBRARY)
endef

$(foreach module,$(PCL_STATIC_LIBRARIES),$(eval $(call build_pcl_static,$(module))))
$(foreach module,$(BOOST_STATIC_LIBRARIES),$(eval $(call build_boost_static,$(module))))
$(foreach module,$(FLANN_STATIC_LIBRARIES),$(eval $(call build_flann_static,$(module))))



include $(CLEAR_VARS)

APP_STL := gnustl_static
OPENCV_LIB_TYPE:=STATIC
NDK_APP_DST_DIR := ../src/main/jniLibs/$(TARGET_ARCH_ABI)


include C:/OpenCV-2.4.10-android-sdk/sdk/native/jni/OpenCV.mk


LOCAL_MODULE :=jni-input
LOCAL_CFLAGS =  -DANDROID_NDK_BUILD -D__STDC_FORMAT_MACROS -D__STDC_INT64__
LOCAL_LDLIBS +=  -llog -ldl
LOCAL_LDLIBS += -L$(SYSROOT)/usr/lib -llog
LOCAL_C_INCLUDES += $(LOCAL_PATH)
JSONCPP_PATH := ../JSON
LOCAL_C_INCLUDES += ${JSONCPP_PATH}
STATE_MODEL_PATH := ../stateModel
LOCAL_C_INCLUDES += ${STATE_MODEL_PATH}

LOCAL_CXXFLAGS += -fopenmp
LOCAL_CFLAGS += -fopenmp
LOCAL_LDLIBS += -llog -fopenmp

LOCAL_LDFLAGS += -L$(PCL_INCLUDE)/lib  \
                 -L$(BOOST_ANDROID_INCLUDE)/lib \
                 -L$(FLANN_INCLUDE)/lib

LOCAL_C_INCLUDES += $(PCL_INCLUDE)/include \
                    $(BOOST_ANDROID_INCLUDE)/include \
                    $(EIGEN_INCLUDE) \
                    $(FLANN_INCLUDE)/include

LOCAL_STATIC_LIBRARIES   += pcl_common pcl_kdtree pcl_octree pcl_sample_consensus \
                            pcl_surface pcl_features pcl_io pcl_keypoints pcl_recognition \
                            pcl_search pcl_tracking pcl_filters pcl_io_ply pcl_ml \
                            pcl_registration pcl_segmentation

LOCAL_STATIC_LIBRARIES   += boost_date_time boost_iostreams boost_regex boost_system \
                            boost_filesystem boost_program_options boost_signals \
                            boost_thread


LOCAL_SHARED_LIBRARIES   += flann flann_cpp

LOCAL_CFLAGS += -std=c++11 -frtti -fexceptions -fopenmp -w

#LOCAL_ALLOW_UNDEFINED_SYMBOLS := true

LOCAL_SRC_FILES :=com_orbbec_NativeNI_NativeGestureDetect.cpp\
                  Htime.cpp\
                  projectorCamera.cpp\
                  sendEvent.cpp\
                  touchInfo.cpp\
                  ${JSONCPP_PATH}/json_reader.cpp\
                  ${JSONCPP_PATH}/json_value.cpp\
                  ${JSONCPP_PATH}/json_writer.cpp\
                  ${STATE_MODEL_PATH}/role.cpp\
                  ${STATE_MODEL_PATH}/state.cpp\
                  ${STATE_MODEL_PATH}/idle.cpp\
                  ${STATE_MODEL_PATH}/singleActive.cpp\
                  ${STATE_MODEL_PATH}/doubleOneHandActive.cpp\
                  ${STATE_MODEL_PATH}/doubleTwoHandActive.cpp\


include $(BUILD_SHARED_LIBRARY)