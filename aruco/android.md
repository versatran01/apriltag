cmake ../trunk/ -DCMAKE_TOOLCHAIN_FILE=/home/salinas/Libraries/android-ndk-r15b/build/cmake/android.toolchain.cmake  -DANDROID_NDK=/home/salinas/Libraries/android-ndk-r15b -DANDROID_NATIVE_API_LEVEL=19 -DANDROID_ABI="armeabi-v7a with NEON" -DOpenCV_DIR=/home/salinas/Descargas/OpenCV-android-sdk/sdk/native/jni  -DBUILD_UTILS=OFF  -DANDROID_STL=gnustl_shared && make VERBOSE=1


check https://developer.android.com/ndk/guides/cmake.html#variables
