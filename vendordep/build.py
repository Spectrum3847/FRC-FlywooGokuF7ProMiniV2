#!/usr/bin/env python3
"""Builds the GokuIMU Java vendordep into a Maven repo at vendordep/repo and writes GokuIMU.json.

No Gradle needed: compiles with the WPILib JDK against the installed WPILib jars.
  python build.py [--wpilib C:/Users/Public/wpilib/2027_alpha7] [--repo-url https://...]

Teams install it by copying GokuIMU.json into their robot project's vendordeps/ folder; the
jar is fetched from the URL in mavenUrls (host repo/ on GitHub Pages, or use a file:// URL).
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import zipfile

VERSION = "0.2.3"
GROUP = "io.github.frcgokuimu"
ARTIFACT = "GokuIMU-java"
WPILIB_VERSION = "2027.0.0-alpha-8"
WPILIB_YEAR = "2027_alpha7"  # what GradleRIO 2027.0.0-alpha-8 checks for
UUID = "cfb2a9bb-465d-40a2-8f4a-ddb926e58d4d"

HERE = os.path.dirname(os.path.abspath(__file__))
DEPS = ["wpilibj", "wpimath", "wpiunits", "wpiutil", "hal", "ntcore", "telemetry"]


def main():
    ap = argparse.ArgumentParser()
    default_wpilib = "C:/Users/Public/wpilib/2027_alpha7" if os.name == "nt" else os.path.expanduser("~/wpilib/2027_alpha7")
    ap.add_argument("--wpilib", default=default_wpilib)
    ap.add_argument("--repo-url", default="https://spectrum3847.github.io/FRC-FlywooGokuF7ProMiniV2/vendordep/repo")
    a = ap.parse_args()

    maven = os.path.join(a.wpilib, "maven", "org", "wpilib")
    cp = []
    for d in DEPS:
        jars = glob.glob(os.path.join(maven, d, f"{d}-java", WPILIB_VERSION, f"{d}-java-{WPILIB_VERSION}.jar"))
        if not jars:
            sys.exit(f"missing WPILib jar for {d} under {maven}")
        cp += jars
    jdk = os.path.join(a.wpilib, "jdk", "bin")
    exe = ".exe" if os.name == "nt" else ""

    build = os.path.join(HERE, "build")
    classes = os.path.join(build, "classes")
    shutil.rmtree(build, ignore_errors=True)
    os.makedirs(classes)
    srcs = glob.glob(os.path.join(HERE, "src", "main", "java", "**", "*.java"), recursive=True)
    r = subprocess.run([os.path.join(jdk, "javac" + exe), "--release", "21", "-Xlint:all,-serial",
                        "-cp", os.pathsep.join(cp), "-d", classes] + srcs)
    if r.returncode:
        sys.exit(r.returncode)

    out = os.path.join(HERE, "repo", *GROUP.split("."), ARTIFACT, VERSION)
    os.makedirs(out, exist_ok=True)
    base = os.path.join(out, f"{ARTIFACT}-{VERSION}")
    with zipfile.ZipFile(base + ".jar", "w", zipfile.ZIP_DEFLATED) as z:
        for root, _, files in os.walk(classes):
            for f in files:
                p = os.path.join(root, f)
                z.write(p, os.path.relpath(p, classes).replace(os.sep, "/"))
    with zipfile.ZipFile(base + "-sources.jar", "w", zipfile.ZIP_DEFLATED) as z:
        src_root = os.path.join(HERE, "src", "main", "java")
        for p in srcs:
            z.write(p, os.path.relpath(p, src_root).replace(os.sep, "/"))
    with open(base + ".pom", "w") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0">
  <modelVersion>4.0.0</modelVersion>
  <groupId>{GROUP}</groupId>
  <artifactId>{ARTIFACT}</artifactId>
  <version>{VERSION}</version>
  <packaging>jar</packaging>
</project>
""")
    meta = os.path.join(HERE, "repo", *GROUP.split("."), ARTIFACT, "maven-metadata.xml")
    with open(meta, "w") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<metadata><groupId>{GROUP}</groupId><artifactId>{ARTIFACT}</artifactId>
<versioning><latest>{VERSION}</latest><release>{VERSION}</release>
<versions><version>{VERSION}</version></versions></versioning></metadata>
""")

    vd = {
        "fileName": "GokuIMU.json",
        "name": "GokuIMU",
        "version": VERSION,
        "uuid": UUID,
        "wpilibYear": WPILIB_YEAR,
        "mavenUrls": [a.repo_url],
        "jsonUrl": a.repo_url.rsplit("/repo", 1)[0] + "/GokuIMU.json",
        "javaDependencies": [{"groupId": GROUP, "artifactId": ARTIFACT, "version": VERSION}],
        "jniDependencies": [],
        "cppDependencies": [],
    }
    with open(os.path.join(HERE, "GokuIMU.json"), "w") as f:
        json.dump(vd, f, indent=2)
        f.write("\n")
    print("OK ->", base + ".jar")


if __name__ == "__main__":
    main()
