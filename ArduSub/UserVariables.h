// UserVariables.h - 用户自定义变量声明
// 在 UserCode.cpp 中使用的用户变量在此文件声明
// 通过 USERHOOK_VARIABLES 宏保护，仅在 config.h 中定义该宏时才启用
// 示例：Wii 相机测试变量（可替换为自己的变量）

#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES

