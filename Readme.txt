Gobinet  20200622 1.0.1
gobinet添加获取MTU并上报给RIL。
gobinet需要和ril配套，否则拨号后解析ip信息会出错

Gobinet  20200622 1.0.0
提供正式版本


Gobinet  20200617 0.0.8
1. 修改gobi未初始化read_wait导致调用gobi_call_status_show引起的crash问题 --李永刚


Gobinet  20200617 0.0.7
1. 将冗余的log使用开关功能关闭显示  袁浩
2. 版本号 0.0.7


Gobinet 0524 0.0.6
   解决：
   QMI操作函数的内存泄露问题 ---- 袁浩
   调整了打印可读性  ---- 袁浩
   修改一处压测出现的gobinet Crash问题 ---- 李永刚
   添加拨号失败后的重试机制 ---- 袁浩
   待解决问题：
   一例压测Crash待分析 --- 永刚


Gobinet 0509,liyonggang 0.0.3
      1、解决agswgobi目录的问题。 ---- 李永刚