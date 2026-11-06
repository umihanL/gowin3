# 触摸接口依据

显示屏：芯路恒 TFT5.0″ V3.0，电容触摸 GT911。

本机资料 `C:/Users/whuzy/Desktop/Gowin/ACG720套餐配套模块资料.zip` 中的：

- `显示屏/4.3寸5寸800x480高清电容触摸屏/用户手册/芯路恒TFT_V3.0电容触摸显示屏用户手册.pdf`
- `显示屏/4.3寸5寸800x480高清电容触摸屏/背板原理图/5寸电容触摸屏.pdf`
- `显示屏/4.3寸5寸800x480高清电容触摸屏/触摸液晶屏模组厂家规格书/5寸 TFT屏资料/gt91x编程指南.pdf`

确认了触摸控制器、屏幕接口信号复用和协议：TP_SCK→CT_SCL，TP_MOSI→CT_SDA，TP_CS→CT_RST，TP_PEN→CT_INT。电容屏通过 RN9 接入，RN8 是电阻屏接口选择。

本机的 `ACG720-138&60K通用管脚分配表.xlsx` 仅列出 TFT 像素、时钟、同步和背光信号，未列出上述四根触摸信号，也未找到 ACG720 主板原理图。

因此主板引脚映射参考了同型号公开工程的约束文件，其注释标明来自 ACG720-138&60K 原理图第 1、11 页：

https://github.com/Vera-loves-program/acg720-vision-robot/blob/HEAD/src/lcd_interaction.cst

读取的文件 blob SHA：`3fefe139995c50314dfbe1125b38262242c47234`。

| 本工程端口 | FPGA 引脚 | 屏幕信号 | FPC 针号（参考工程标注） |
| --- | --- | --- | --- |
| TP_RST_N | Y21 | TP_CS / CT_RST | 35 |
| TP_SDA | T20 | TP_MOSI / CT_SDA | 36 |
| TP_SCL | T21 | TP_SCK / CT_SCL | 38 |
| TP_INT | P20 | TP_PEN / CT_INT | 39 |

CPU 复用脚必须启用 GPIO 功能：`set_option -use_cpu_as_gpio 1`。原工程 SSPI GPIO 配置也保留。SDA/SCL 使用开漏输出，INT 完成上电选址后释放为高阻，INT 不启用上下拉。

该映射已通过目标器件管脚合法性检查；尚未在用户实物上验证，不能把管脚合法或仿真通过等同于实物触摸已验证。若画面正常但触摸无反应，优先核对这四根信号及屏幕背板是否为电容触摸配置。

驱动采用 100 kHz I²C，选址 0x5D（8 位 0xBA/0xBB），NACK 时尝试另一地址 0x14。读取 0x814E 就绪位及触点数，再读取 0x8150～0x8153 的小端坐标；最后写零清除 0x814E。每 5 ms 轮询，松手也清状态。
