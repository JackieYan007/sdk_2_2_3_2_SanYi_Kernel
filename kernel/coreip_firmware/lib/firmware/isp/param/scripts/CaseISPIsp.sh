INIT_BASE=$(( 0xc0000000 + 0 ))
CMDP_BASE=$(( 0xc0020800 + 0 ))
SLAB_BASE=$(( 0xc0300000 + 0 ))
FBUF_BASE=$(( 0xc0400000 + 0 ))
ISP_BASE_HIGH=1
PATH_RODATA=${ASIC_SW_ROOT}/ucode/uc_sdk.isp/master/build/src/exe/isp/core_isp.exe.rodata.bin
PLATFORM="F440"
ENV="XS18"
TRACEMASK=0xffffffff

WIDTH=1280
HEIGHT=720
FMT=0      #0:NV12

GlbCfgFile="GlbCfgParam.bin"
Snr00CfgFile="SnrCfgParam.bin"
Alg00CfgFile="AlgoParam.bin"
Snr01CfgFile="SnrCfgParam.bin"
Alg01CfgFile="AlgoParam.bin"
Snr02CfgFile="SnrCfgParam.bin"
Alg02CfgFile="AlgoParam.bin"
Snr03CfgFile="SnrCfgParam.bin"
Alg03CfgFile="AlgoParam.bin"
Snr04CfgFile="SnrCfgParam.bin"
Alg04CfgFile="AlgoParam.bin"
Snr05CfgFile="SnrCfgParam.bin"
Alg05CfgFile="AlgoParam.bin"
Snr06CfgFile="SnrCfgParam.bin"
Alg06CfgFile="AlgoParam.bin"
Snr07CfgFile="SnrCfgParam.bin"
Alg07CfgFile="AlgoParam.bin"
Snr08CfgFile="SnrCfgParam.bin"
Alg08CfgFile="AlgoParam.bin"
Snr09CfgFile="SnrCfgParam.bin"
Alg09CfgFile="AlgoParam.bin"
Snr10CfgFile="SnrCfgParam.bin"
Alg10CfgFile="AlgoParam.bin"
Snr11CfgFile="SnrCfgParam.bin"
Alg11CfgFile="AlgoParam.bin"
VinCfgFile="VinCfgParam.bin"

INIT_ARGS="${INIT_BASE} ${CMDP_BASE} ${SLAB_BASE} ${FBUF_BASE} ${ISP_BASE_HIGH} ${PATH_RODATA} ${PLATFORM} ${ENV} ${TRACEMASK} 0"
USER_ARGS="${WIDTH} ${HEIGHT} ${FMT}"
FILES_ARGS="${GlbCfgFile} ${Snr00CfgFile} ${Alg00CfgFile} ${Snr01CfgFile} ${Alg01CfgFile} ${Snr02CfgFile} ${Alg02CfgFile} ${Snr03CfgFile} ${Alg03CfgFile} ${Snr04CfgFile} ${Alg04CfgFile} ${Snr05CfgFile} ${Alg05CfgFile} ${Snr06CfgFile} ${Alg06CfgFile} ${Snr07CfgFile} ${Alg07CfgFile} ${Snr08CfgFile} ${Alg08CfgFile} ${Snr09CfgFile} ${Alg09CfgFile} ${Snr10CfgFile} ${Alg10CfgFile} ${Snr11CfgFile} ${Alg11CfgFile} ${VinCfgFile}"

if [[ $(uname -s) == Linux ]];then
CMD="${PATH_SCRIPTS}/genProtoAPIIsp ${INIT_ARGS} ${USER_ARGS} ${FILES_ARGS}"
else
CMD="${PATH_SCRIPTS}/genProtoAPIIsp.exe ${INIT_ARGS} ${USER_ARGS} ${FILES_ARGS}"
fi

echo ${CMD}

${CMD}
