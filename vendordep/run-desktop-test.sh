#!/bin/bash
# Desk test of the vendordep with WPILib desktop (simulation HAL) natives on Windows.
# Runs the simulation tests, plus hardware tests if a port is given:
#   bash run-desktop-test.sh          # sim only
#   bash run-desktop-test.sh COM6     # sim + real IMU (auto-discovered)
# Run `python build.py` first.
cd "$(dirname "$0")"
WPI=${WPILIB:-/c/Users/Public/wpilib/2027_alpha7}
J=$WPI/jdk/bin; MV=$WPI/maven; V=2027.0.0-alpha-8
mkdir -p build/natives
for d in hal wpiutil wpimath wpinet ntcore datalog; do
  unzip -oqj "$MV/org/wpilib/$d/$d-cpp/$V/$d-cpp-$V-windowsx86-64.zip" "windows/x86-64/shared/*.dll" -d build/natives
done
CP="build/classes"
for d in wpilibj wpimath wpiunits wpiutil hal ntcore telemetry datalog wpinet tunables; do
  j="$MV/org/wpilib/$d/$d-java/$V/$d-java-$V.jar"; [ -f "$j" ] && CP="$CP;$(cygpath -w "$j")"
done
for j in $(find "$MV/us" "$MV/org/ejml" "$MV/com/fasterxml" -name "*.jar" ! -name "*sources*" ! -name "*javadoc*" 2>/dev/null); do
  CP="$CP;$(cygpath -w "$j")"
done
rm -rf build/test
"$J/javac" -cp "$CP" -d build/test src/test/java/io/github/frcgokuimu/FullTest.java || exit 1
PATH="$(pwd)/build/natives:$PATH" "$J/java" --enable-native-access=ALL-UNNAMED \
  -Djava.library.path=build/natives -cp "$CP;build/test" io.github.frcgokuimu.FullTest "$@"
