Import("env", "projenv")
platform = env.PioPlatform()
env.Prepend(
    UPLOADERFLAGS=["-s", platform.get_package_dir("tool-openocd")+"/scripts"]
)
env.Append(
    UPLOADERFLAGS=["-f", "./upload.cfg"]
)
env.Append(
    UPLOADERFLAGS=["-c", "program {$SOURCE} 0x08000000 verify; reset init; resume; shutdown"] 
)
env.Replace(
    UPLOADER="openocd",
    UPLOADCMD="$UPLOADER $UPLOADERFLAGS"
)