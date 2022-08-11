PATH_SCRIPTS=../param/scripts

perl ${PATH_SCRIPTS}/GlbParam2Bin.pl GlbCfg_Param.txt

INIT_BASE=$(( 0xc0000000 + 0 ))
CMDP_BASE=$(( 0xc0020800 + 0 ))
SLAB_BASE=$(( 0xc0300000 + 0 ))
FBUF_BASE=$(( 0xc0400000 + 0 ))
ISP_BASE_HIGH=1
PATH_RODATA=${PATH_SCRIPTS}/core_isp.exe.rodata.bin
PLATFORM="F440"
ENV="XS18"
TRACEMASK=0xffffffff

WIDTH=1280
HEIGHT=720
FMT=0      #0:NV12

GlbCfgFile="GlbCfgParam.bin"
Snr00CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg00CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr01CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg01CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr02CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg02CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr03CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg03CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr04CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg04CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr05CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg05CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr06CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg06CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr07CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg07CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr08CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg08CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr09CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg09CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr10CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg10CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
Snr11CfgFile="${PATH_SCRIPTS}/SnrCfgParam.bin"
Alg11CfgFile="${PATH_SCRIPTS}/AlgoParam.bin"
VinCfgFile="./VinCfgParam.bin"

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
