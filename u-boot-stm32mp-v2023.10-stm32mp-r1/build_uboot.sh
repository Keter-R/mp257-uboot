#!/bin/bash

#判断交叉编译工具链是否存在
if [ ! -e "/opt/st/stm32mp2/5.0.3-snapshot/environment-setup-cortexa35-ostl-linux" ]; then
    echo "未在该路径下找到交叉编译器:/opt/st/stm32mp2/5.0.3-snapshot/"
    echo "请先安装交叉编译器:atk-image-openstlinux-weston-stm32mp2.rootfs-x86_64-toolchain-5.0.3-snapshot*.sh"
    exit 1
fi

#配置交叉编译器
source /opt/st/stm32mp2/5.0.3-snapshot/environment-setup-cortexa35-ostl-linux

#检查输入内存容量参数: 1:DDR_1GB; 2:DDR_2GB
if [ $# -eq 0 ]; then
    read -p "请选择DDR内存容量, 输入数字1或2, 按Enter键确认, 开始编译:
1.DDR_1GB
2.DDR_2GB
输入数字: " ddr_size_number
    if [ $ddr_size_number -ne 1 ] && [ $ddr_size_number -ne 2 ]; then
        echo "error:unsupport"
        exit 1
    fi
else 
    echo "usage example:./build_uboot.sh"
    exit 1
fi

#导出相关环境变量
export FIP_DEPLOYDIR_ROOT=$PWD/../../FIP_artifacts

#编译前先清理上一次的编译结果
if [ -e $PWD/../build ];then
    rm -rf $PWD/../build
fi

#编译前先删除上一次编译的部署镜像文件
uboot_deploydir=$FIP_DEPLOYDIR_ROOT/u-boot
if [ -e "$uboot_deploydir" ]; then
    rm -rf $uboot_deploydir
fi

#指定要编译的设备树文件
#DDR 1GB设备树
devicetree_ddr_1GB=stm32mp257d-atk-ddr-1GB
#DDR 2GB设备树
devicetree_ddr_2GB=stm32mp257d-atk-ddr-2GB

#编译uboot源码
if [ $ddr_size_number -eq 1 ]; then
    #编译DDR 1GB配置
    make -f $PWD/../Makefile.sdk UBOOT_DEFCONFIG=stm32mp25_defconfig DEVICE_TREE=$devicetree_ddr_1GB DEPLOYDIR=$FIP_DEPLOYDIR_ROOT/u-boot all
elif [ $ddr_size_number -eq 2 ]; then
    #编译DDR 2GB配置
    make -f $PWD/../Makefile.sdk UBOOT_DEFCONFIG=stm32mp25_defconfig DEVICE_TREE=$devicetree_ddr_2GB DEPLOYDIR=$FIP_DEPLOYDIR_ROOT/u-boot all
else
    #非正确编译选项
    echo "error:unsupport"
    exit 1
fi

#提示编译结束，请查看具体编译信息
echo "---Compile finish---"
