
#include <android/log.h>
#include <jni.h>
#include "BMI160_IMU.h"

std::shared_ptr<BMI160_IMU> sensor = nullptr;
JavaVM* vm;
JNIEnv* jni;
jclass mainActivityClass;
jobject mainActivity = nullptr;
jmethodID onSensorChanged;

void sensorCallback(float accx, float accy, float accz, float gyrx, float gyry, float gyrz, long ts) {
    if (!mainActivity || !jni || !onSensorChanged) {
        __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "env, mainActivity or methodID was null!");
        return;
    }

    jni->CallVoidMethod(mainActivity, onSensorChanged, accx, accy, accz, gyrx, gyry, gyrz, ts);
}

void sensorSetup() {
    vm->AttachCurrentThread(&jni, 0);
    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Attached IMU_BMI thread.");
}

void sensorTeardown() {
    vm->DetachCurrentThread();
    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Detached IMU_BMI thread.");
}

extern "C"
JNIEXPORT void JNICALL
Java_com_r1k_utilities_allanvariancesampler_MainActivity_teardown(JNIEnv *env, jobject instance) {

    sensor->stop();
    sensor = nullptr;

    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Goodbye!");
}


extern "C"
JNIEXPORT void JNICALL
Java_com_r1k_utilities_allanvariancesampler_MainActivity_setup(JNIEnv *env, jobject instance) {

    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Hello.");

    jni = env;
    env->GetJavaVM(&vm);
    mainActivity = reinterpret_cast<jclass>(env->NewGlobalRef(instance));
    mainActivityClass = jni->GetObjectClass(mainActivity);
    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Got class.");

    onSensorChanged = jni->GetMethodID(mainActivityClass, "onSensorChangedBMI160", "(FFFFFFJ)V");
    __android_log_print(ANDROID_LOG_DEBUG, "ALLAN_VARIANCE_SAMPLER_NATIVE", "Got method ID.");

    sensor = std::make_shared<BMI160_IMU>(std::bind(&sensorSetup),
                                          std::bind(&sensorTeardown),
                                          std::bind(&sensorCallback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));



    sensor->start();

}