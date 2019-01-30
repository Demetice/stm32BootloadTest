# stm32f103zet6 的bootload
- master 分支是 app程序
- br_bootload 分支是 bootload

## 如何使用
1. 先编译下载bootload
2. 编译app  得到bin 文件
3. 串口连接单片机的USART1,
4. 打开上位机选择bin文件 下载

## 如何验证
1. 请查看我的个人仓库，有个上位机项目
2. 下载下来，安装QT 5.10以上版本
3. 编译运行

[更多细节看doc内文件](./doc/stm32f103 bootload规划.md)
