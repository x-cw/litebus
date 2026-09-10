Litebus Motivation、Feature、Performance
版本：v1.1 作者：chenqw
【说明】本版基于目前方案讨论、实验结果结论进行总结，不代表最终版本与验收质量。
一、Motivation（动机）
GP3A 项目采用Arteris通用总线 FlexNoC 构建HBF(High-Bandwidth Fabric)，用于承载片中高负载
通信，尤其GPC与L2 Cache的互联。目前，存在以下痛点：
⚫ 冗余的协议转化开销：GPC core的原生接口是BIF，FlexNoC内部使用AXI5协议做桥接。协议转
换带来额外的延迟、面积延迟开销，并导致FlexNoC NIU(总线准入接口)、GPC BIF2AXI接口臃
肿。一种“看起来可能解决”的BIF2NSP方案也被证实无法应用：NSP是FlexNoC可选择的外部
socket接口，与BIF接口高度适配，然而由于FlexNoC固有协议限制，无法同时支持多
Outstanding Atomic与NSP接口（两者互斥）。
⚫ 同ID处理的硬性、不可配置消除逻辑：FlexNoC以通用为买点，原生必须支持“同一MATSER向
不同SLAVE发送同ID transaction，并保证与AXI一致的同ID保序语义”，这引入大量用于“同
ID检查、记录、反压、释放、重分配”的逻辑，包含查找表等CAM（Context Address
Memory，内容查找存储器）、并行比较矩阵、多级归并树等复杂逻辑，这一部分是NIU的时序收
敛痛点。然而这一部分并不是强制必须的，参考GP3 GPC到总线的事务pattern，并不会发送同
ID事务，因此，FlexNoC相关的同ID逻辑没有意义，造成无意义的时序、面积消费。
⚫ 无法绕开物理通道对带宽的强制限制：GP3A的FlexNoC总线只有两个通道（REQ、RSP）。REQ
同时承载读写请求，RSP则是读写响应。同一物理通道读写分时复用带来的竞争，导致读写带宽无
法得到有效提升。理论上，随着Burst_len增大 ，有效数据量（写请求Wdata、读响应Rdata）会
越来越大于非数据开销（单拍的读CMD、写响应）。然而目前这里有两个限制，一是GPC内部的
Cacheline粒度决定了“2拍”burst长度，导致理论上限带宽也不会超过2/3的双向带宽；二是即
使Burst_len可以无限增加，长burst事务经过Switch仲裁（Switch必须以事务为原子粒度，不
可打断）时，HOL阻塞（Head of Line Blocking）会导致Switch输出端口空转增加，进一步降低
总带宽。一种“看起来可能解决”的FlexNoC读写拆开方案也被证实无法应用：NIU读写分离会导
致Atomic无法使用，NIU面积几乎翻倍。
⚫ Valid-Ready流控机制的实现难度大：FlexNoC内部的NIU、LINK、Switch等Object之间连接
仍然采用Vld-rdy流控。Object在Floorplan有独立的region坐标，而Vld-rdy在正反向上都有
时序牵扯，因此在长距离走线过程中，需要使用skid_buffer精确打拍，且需要后端开辟足够的走
线通道容纳skid_buffer、和进行布局调整pipe位置。Vld-rdy流控所在的布线通道往往会成为
Congestion热点，进而撑大原来的通道面积。

⚫ 不支持广/组播、规约等扩展：FlexNoC是标准的非一致性总线，不支持扩展广播、规约等功能。目
前，HPC上若支持扩展该功能，会减少业务中副本复制、迁移的次数。该部分TBD，需要详细讨论
需求。
⚫ 黑盒特性：FlexNoC作为一个商业IP，其内部结构、RTL coding、内部协议都没有公开。定制化需
求需要对FlexNoC进行白盒化，难度较高，风险较大。
基于上述 GP3 FlexNoC 实际落地的痛点问题，本文致力研发一款与 INNO GPU 深度适配的定制化、轻
量化、完全白盒的总线，解决以下问题：
⚫ 高度适配的Litebus接口协议：该接口与BIF适配，缩减协议开销，辅以AXI2LB桥来兼容其他
AXI接口 IP。
⚫ 同ID分离、简化：处理“同ID transaction”问题不再作为强制必选，可大幅简化同ID保序逻
辑。同ID处理被迁移至Reorder Buffer或者其他“查表反压”等独立、可选组件、MASTER IP
端。
⚫ 四通道分离：读写通道独享一个物理通道带宽，消除竞争引入的带宽损失。通道分离带来的同一个
MASTER的“RR、WW、RW、WR”保序机制，与AXI语义一致。MASTER IP需要继承GP3的
读写冲突检测机制，总线同样不保证读写间的保序问题。
⚫ 引入Credit-Based流控：首次尝试引入Credit机制，旨在解决VLD-RDY双向时序牵扯、精简打
拍逻辑、面积热点重布局（可减少走线）通道等问题。
⚫ 广/组播扩展：待定，需要专题讨论。LITEBUS的白盒化开发使得广播、组播、规约等复杂需求实现
提供基石。

二、Overall（总框架）
LITEBUS总线规划始于两条基线：
⚫ LITEBUS-BASIC-CORE：总线基础核心，剥离复杂需求，追求高频率、高带宽的极致性能表现，
用于承担GPC到L2 Cache的高负载互联需求。
⚫ LITEBUS-EXPENDED-CORE：总线扩展组件，在BASIC-CORE的基础上，扩展开发组件，用于兼
容其他AMBA协议的IP流，可按照需求配置。该组件集包含较为复杂的查找表、并行比较、归并
等复杂、拖累主频的逻辑链，需与高频BASIC-CORE分开设计，解耦。
2.1 LITEBUS-BASIC-CORE
2.1.1 组件构成
BASIC-CORE 包含 Initiator NIU、Target NIU、Switch、Link 等基础组件，根据用户需求进行 TOPO
完成总线结构搭建。
⚫ Initiator NIU：发起侧 NIU，负责事务请求入网与事务响应出网、地址编码、total_bytes 有效总字
节计算、Local_Address计算、SrcID 注入、vld-rdy与credit 流控转化。
⚫ Target NIU：目标NIU，负责事务请求出网网与事务响应入网、CMD TABLE分配与释放、SLAVE
侧事务适配、读交织处理、vld-rdy与credit 流控转化。
⚫ Switch：网络路由器，负责事务级传输、XBAR-FIFO部署、仲裁、credit 流控。
⚫ Link：网络串联器，负责网络打拍（pipe）、跨异步（bca）、增加credit节点（node）、上下游
Flit包位宽转化（unify）。
2.1.2 数据流
写事务的主要数据流如下，
W1：MASTER IP发送LITEBUS写请求，可经过pipe-vd(ext)打拍后进入INIU内部。
W2：写请求抽离Address字段，进入ID decode，做地址编码获取目标ID（Destid）。
W3：完成 Srcid 注入、有效总字节（total_byte）计算、目标偏移地址（local_address）计算，经过拼
接后输出Flit格式写请求。
W4：通过Credit_Egress，将vld-rdy流控转化为credit流控，发送至下游，可经过pipe-cr打拍。
W5：经过Fabric中Switch、Link组件路由至TNIU。
W6：写请求可通过pipe-cr打拍进入TNIU内部，并送至Credit Ingress的FIFO中，转化为vld-rdy，
供TNIU内部获取。
W7：写请求通过CMD Table，被分配CMD槽，存储Srcid、txnid等关键字段，用于响应回程查表。
W8：获取CMD Table的槽索引，作为下游的SLAVE IP接口的txnid。
W9：写请求通过lane pack，完成目标写命令转换、写数据相位对齐、STRB补齐。

W10：最终写请求可选择打拍pipe-vd(ext)打拍后，输出至SLAVE。
W11：写响应可选择打拍pipe-vd(ext)打拍后进入TNIU。
W12：使用回环的txnid释放CMD TABLE槽。
W13：通过Credit_Egress，将vld-rdy流控转化为credit流控，发送至下游，可经过pipe-cr打拍。
W14：经过Fabric中Switch、Link组件路由至INIU。
W15：写响应可通过pipe-cr 打拍进入INIU 内部，依次pipe-cr打拍、Credit Ingress 的FIFO、pipe-
vd(ext)，输出至MASTER IP。
读事务的主要数据流如下，
R1：MASTER IP发送LITEBUS读请求，可经过pipe-vd(ext)打拍后进入INIU内部。
R2：读请求抽离Address字段，进入ID decode，完成目前地址编码，获得目标ID（Destid）。
R3：完成 Srcid 注入、有效总字节（total_byte）计算、目标偏移地址（local_address）计算，经过拼
接后输出Flit格式写请求。
R4：通过Credit_Egress，将vld-rdy流控转化为credit流控，发送至下游，可经过pipe-cr打拍。
R5：经过Fabric中Switch、Link组件路由至TNIU。
R6：读请求可通过pipe-cr打拍进入TNIU内部，并送至Credit Ingress的FIFO中，转化为vld-rdy，
供TNIU内部获取。
R7：读请求通过CMD Table，被分配CMD槽，存储Srcid、txnid、total_byte、addr_lo等关键字段，
用于响应回程查表。
R8：获取CMD Table的槽索引，作为下游的SLAVE IP接口的txnid。
R9：读请求通过cmd conv，完成目标写读命令转换。
R10：最终读请求可选择打拍pipe-vd(ext)打拍后，输出至SLAVE。
R11：读响应可选择打拍pipe-vd(ext)打拍后进入TNIU。
R12：使用回环的txnid释放CMD TABLE槽。
R13：读响应（支持读交织）经过rd flag，切分beat为事务，其中的assembly用于小位宽到大位宽的
fragment收集。
R14：通过Credit_Egress，将vld-rdy流控转化为credit流控，发送至下游，可经过pipe-cr打拍。
R15：经过Fabric中Switch、Link组件路由至INIU。
R16：读响应可通过 pipe-cr 打拍进入 INIU 内部，依次 pipe-cr 打拍、Credit Ingress 的 FIFO、pipe-
vd(ext)，输出至MASTER IP。
2.1.3 Feature
BASIC-CORE支持以下Feature，
⚫ 支持自定义LITEBUS接口协议
◼ Valid-ready流控接口

| ◼  事务类型  |     |     |     |
| -------- | --- | --- | --- |
◆  读Burst，only INCR
◆  写Burst，only INCR
◆  Atomic
⚫  Atomic Store
⚫  Atomic Load
⚫
Atomic Swap
⚫  Atomic Compare
| ◼  地址位宽：8~64 bit                                             |     |     |     |
| ------------------------------------------------------------ | --- | --- | --- |
| ◼  突发长度位宽：1~12 bit                                           |     |     |     |
| ◼  ID位宽：1~16 bit，2^n等于MAX Pendingtrans，Pendingtrans所有事务类型共用  |     |     |     |
| ◼  REQ_R/REQ_W/RD通道user位宽独立配置：1~64 bit                       |     |     |     |
| ◼  数据位宽：8~1024 bit，2^n                                       |     |     |     |
| ◼  写数据支持STRB掩码，数据位宽/8                                        |     |     |     |
| ◼  原子修饰符 req_w_modifier位宽：0~7 bit                            |     |     |     |
⚫
支持读响应交织
⚫  支持非对齐传输
⚫  仅支持MASTER发送不同ID，不能发送同ID请求（读写通道分开计算、范围包括写写事务、读读
事务、原子操作与正常事务），若要支持同ID请求，需使用EXTENDED_CORE中的扩展组件。此
前提下，LITEBUS不向SLAVE IP发同ID请求
⚫  仅支持MASTER 的total_burst_size（(len+1）*data）小于SLAVE，MATSER IP不能发送大于
SLAVE burst_size上限的事务包。若要支持，需要使用EXTENDED_CORE中的扩展组件，进行拆
包
⚫  Object互联使用credit-based流控机制
⚫
支持不同MASTER、SLAVE的参数位宽统一
⚫  支持Switch XBAR FIFO缓解HOL阻塞
⚫  四通道分离，提高物理带宽上限，RR/WW保序，RW不保序（需MASTER IP做address overlap
检查）
⚫
Switch支持Round-Robin、Fix仲裁机制
⚫  支持四种类似的Link：PIPE、NODE、BCA、Unify

2.1.4 Interface
时钟复位：
| 信号名  | 方向       | 位宽  | 说明              |
| ---- | -------- | --- | --------------- |
| clk  | TOP→NIU  | 1   | LITEBUS_CORE时钟  |

| rst_n  | TOP→Master  | 1   | LITEBUS_CORE | 复位，要求顶层做同步复位释 |
| ------ | ----------- | --- | ------------ | ------------- |
放CRM
REQ_W通道：
| 信号名          | 方向  | 位宽  | 说明                     |     |
| ------------ | --- | --- | ---------------------- | --- |
| req_w_valid  |     | 1   | Valid-Ready握手，Valid信号  |     |
Master→INIU
TNIU→Slave
| req_w_ready  | INIU→Master  | 1   | Valid-Ready握手，Ready信号  |     |
| ------------ | ------------ | --- | ---------------------- | --- |
Slave→TNIU
| req_w_opcode  | Master→INIU  | 4   | 命令码。        |     |
| ------------- | ------------ | --- | ----------- | --- |
|               | TNIU→Slave   |     | 4’h8：普通写操作  |     |
4’hC：Atomic Store
4’hD：Atomic Load
4’hE：Atomic Swap
4’hF：Atomic Compare
其他值非法，待启用，预留广播、组播、规约
| req_w_addr  |              | 8-64  |                      |     |
| ----------- | ------------ | ----- | -------------------- | --- |
|             | INIU→Master  |       | Master侧：全局地址         |     |
|             | Slave→TNIU   |       | Slave侧：local 地址      |     |
| req_w_len   |              | 1-12  | 拍数 − 1（len=0 → 1 拍）  |     |
INIU→Master
Slave→TNIU
req_w_txnid  INIU→Master  1-16  外部事务 ID。TNIU 侧即 LID，Slave 必须原
|             | Slave→TNIU   |       | 样回传，否则回程还原错  |     |
| ----------- | ------------ | ----- | ------------ | --- |
| req_w_user  | INIU→Master  | 1-64  | 用户边带         |     |
Slave→TNIU
|  req_w_wdata  | INIU→Master  | 8~1024  | 写数据，2^n  |     |
| ------------- | ------------ | ------- | -------- | --- |
Slave→TNIU
|  req_w_wstrb  | INIU→Master  | DATA_WIDTH/8 写字节使能  |     |     |
| ------------- | ------------ | ------------------- | --- | --- |
Slave→TNIU
|  req_w_modifier  | INIU→Master  | 0~7  | ATOMIC修饰符  |     |
| ---------------- | ------------ | ---- | ---------- | --- |
Slave→TNIU
| req_w_mcast_mask INIU→Master  |     | TBD  | 组播/广播目标掩码，预留   |     |
| ----------------------------- | --- | ---- | -------------- | --- |
Slave→TNIU

REQ_R通道：
| 信号名          | 方向           | 位宽  | 说明                     |
| ------------ | ------------ | --- | ---------------------- |
| req_r_valid  | Master→INIU  | 1   | Valid-Ready握手，Valid信号  |
TNIU→Slave
| req_r_ready  | INIU→Master  | 1   | Valid-Ready握手，Ready信号  |
| ------------ | ------------ | --- | ---------------------- |
Slave→TNIU
| req_r_opcode  | Master→INIU  | 4   | 命令码。        |
| ------------- | ------------ | --- | ----------- |
|               | TNIU→Slave   |     | bit3=0：读操作  |
其他值非法，待启用，预留值广播、组播、规
约
| req_r_addr  | Master→INIU  | 8-64  | Master侧：全局地址         |
| ----------- | ------------ | ----- | -------------------- |
|             | TNIU→Slave   |       | Slave侧：local 地址      |
| req_r_len   | Master→INIU  | 1-12  | 拍数 − 1（len=0 → 1 拍）  |
TNIU→Slave
req_r_txnid  Master→INIU  1-16  外部事务 ID。TNIU 侧即 LID，Slave 必须原
|             | TNIU→Slave   |       | 样回传，否则回程还原错  |
| ----------- | ------------ | ----- | ------------ |
| req_r_user  | Master→INIU  | 1-64  | 用户边带         |
TNIU→Slave

| req_r_mcast_mask Master→INIU  |     |     | 组播/广播目标掩码，预留   |
| ----------------------------- | --- | --- | -------------- |

TNIU→Slave

RD通道：
| 信号名       | 方向           | 位宽  | 说明                     |
| --------- | ------------ | --- | ---------------------- |
| rd_valid  | TNIU→Master  | 1   | Valid-Ready握手，Valid信号  |
Slave→INIU
| rd_ready  | Master→INIU  | 1   | Valid-Ready握手，Ready信号  |
| --------- | ------------ | --- | ---------------------- |
TNIU→Slave
| rd_last  | TNIU→Master  | 1   | 读 burst 末拍  |
| -------- | ------------ | --- | ----------- |
Slave→INIU
| rd_resp  | TNIU→Master  | 2   | 响应码  |
| -------- | ------------ | --- | ---- |
Slave→INIU
2’b00：OK
2’b10：FAIL
 其他值预留
| rd_txnid  | TNIU→Master  | 1-16  | 外部事务 ID  |
| --------- | ------------ | ----- | -------- |
Slave→INIU
| rd_user  | TNIU→Master  | 1-64  | 用户边带  |
| -------- | ------------ | ----- | ----- |
Slave→INIU

WR通道：
| 信号名       | 方向           | 位宽  | 说明                     |
| --------- | ------------ | --- | ---------------------- |
| wr_valid  | INIU→Master  | 1   | Valid-Ready握手，Valid信号  |
Slave→TNIU
| wr_ready  | Master→INIU  | 1   | Valid-Ready握手，Ready信号  |
| --------- | ------------ | --- | ---------------------- |
TNIU→Slave
| wr_resp  |              | 2   |           |
| -------- | ------------ | --- | --------- |
|          | INIU→Master  |     | 响应码       |
|          | Slave→TNIU   |     | 2’b00：OK  |
2’b10：FAIL
 其他值预留
| wr_txnid  | INIU→Master  | 1-16  | 外部事务 ID  |
| --------- | ------------ | ----- | -------- |
Slave→TNIU
| wr_user  | INIU→Master  | 1-64  | 用户边带  |
| -------- | ------------ | ----- | ----- |
Slave→TNIU

2.1.5 端到端时序（普通事务）

2.1.6 端到端时序（原子事务）

2.2 LITEBUS-EXTENDED-CORE
2.2.1 组件构成（待细化）
（1）AXI/APB-to-LiteBus Adaptor 、LiteBus-to-AXI/APB Adaptor：用于连接AMBA接口的IP。
（2）Narrow burst merge：部分 IP（PCIE）会发出 narrow burst，而 LITEBUS 原生不带类似 AXI-
size字段，需要在进BASIC-CORE之前，将Narrow burst压缩，并解压响应。
（3）Burst split：MATSER IP发送事务大小超过SLAVE IP最大burst size，则需要拆包。为保序，拆
包后的子事务必须是同 ID，然而 LITEBUS 不支持同 ID，该项必须配合 reorder buffer 或者
One_trans_fly使用。
（4）One_trans_fly：内置Pendingtrans深度的Cxt表，用于存储txnid。若新来的事务txnid命中表
中 txnid，则反压。如此，可保证 BASIC-CORE 网络中只有一笔同 ID。该模块适用于性能需求较低的
MATSER。
（5）Reorder Buffer：内置ID重映射、响应重组等复杂逻辑，可将同ID请求重映射成不同ID请求并
发，响应重组后按原ID顺序发回。适用于性能要求较高的Matser IP，然而需要注意，ROB逻辑应该不
会简单，会成为时序瓶颈。
（6）Simple burst split：作为Burst split的简化版，放置在SLAVE一侧。若SLAVE IP支持同ID，
则只需要在请求侧TNIU出口，保持wdata切分len，划分子事务请求；并在响应侧处理子事务带来的多
笔响应问题即可。
2.2.2 Master IP行为分析
按照目前GP3A的MASTER行为，到FlexNoC INIU接口处时，行为已经约束：
⚫ 同ID行为：无Master IP发送同ID事务，IP内部可自行处理。
⚫ Narrow Burst：pcie、vdec、scp。
⚫ 拆包需求：按照项目实际需求变动，倾向与“倾向于调大wlen，使得数据位宽较小的SLAVE的
max burst size等于MATSER实际发出的最大值”。
以上EXTENEDED需求，需按照实际项目需求，与IP进行磋商，决定在IP侧还是总线侧解决。

三、Performance、Area、Timing（基于 BASIC CORE）
3.1 DEMO 与测试环境
本章使用16x16s8d8作为demo测试，配置如下
⚫ 16个Master、16个Slave
⚫ 二级Switch Xbar接口，与GP3的XBAR_L1相似
⚫ 数据位宽512 bit
⚫ Slave MAX Pendingtrans 256
⚫ 每个Switch Xbar FIFO：8x512 bit
⚫ Credit Fifo：4x512 bit
⚫ Atomic暂未实现
测试环境：
⚫ 每个Master背靠背发送读写混合事务，读写各5000笔，压力给满。
⚫ 每个Slave无响应延迟回复，读交织回复。
⚫ 突发长度为2拍。
⚫ 带宽统计窗口：
◼ 写事务：每个MASTER的起始时间为“REQ_W通道发出的第一笔第一拍写数据”时刻，结束
时间为“REQ_W通道接收到最后一笔最后一拍的写数据”时刻。
◼ 读事务：每个MASTER的起始时间为“RD通道收到的第一笔第一拍读数据”时刻，结束时间
为“RD通达接收到最后一笔最后一拍的读数据”时刻。

3.2 Performance（带宽）

3.3 Area、Timing
DC综合环境
1.8G时钟（REG2REG无VIO）：

2.5G时钟（REG2REG VIO <10ps）：