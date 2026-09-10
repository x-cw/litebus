Litebus Principle
版本：v1.2 作者：chenqw
【说明】本版基于目前方案讨论、实验结果结论进行总结，不代表最终版本与验收质量。
一、Principle（原则）
LITEBUS的极简设计原则，贯穿该总线的BASIC与EXPANDEND规划落地。
1.1 相位型事务定位标
互联总线的本质特性是“访问”，即在统一地址空间中，赋予MASTER“访问SLAVE指定地址空间”的
能力，因此，首要一件事是定义“这段指定地址空间”。这个定义需要具备：
| ⚫   | 以Byte作为基本地址单位  |     |     |     |     |
| --- | -------------- | --- | --- | --- | --- |
⚫
描述一个范围、非碎片化的连续空间
| ⚫   | 总线对外端口尽量匹配现有BIF、AXI语义                   |     |     |     |     |
| --- | --------------------------------------- | --- | --- | --- | --- |
| ⚫   | 总线对内接口经过自定义拓扑中任意参数、任意形式的位宽转换，仍然不丢失范围信息  |     |     |     |     |
| ⚫   | 位宽转换器尽量简单、灵活，以适配复杂的总线拓扑需求               |     |     |     |     |

为满足上述原则，外部接口采用与AXI相似、但仍然存在差异的语义：
| ⚫   | Address：字节对齐的地址，支持任意非对齐地址的承载                        |     |     |     |     |
| --- | --------------------------------------------------- | --- | --- | --- | --- |
| ⚫   | Len：突发事务长度，指示当前事务的突发长度(len+1)                       |     |     |     |     |
| ⚫   | 不再设置Size信号，窄带传输统一由EXTENDED_CORE专门处理，再将符合BASIC_CORE语 |     |     |     |     |
义的事务下发

| 总线内部则一律使用“相位型事务定位标”。Initiator NIU |     |     | 将外接口请求携带的 |     | Address、Len，转化 |
| --------------------------------- | --- | --- | --------- | --- | -------------- |
为内接口Lane、Total_bytes。
𝐿𝑎𝑛𝑒 = 𝑆𝑙𝑖𝑐𝑒(𝐴𝑑𝑑𝑟𝑒𝑠𝑠)
|                           |                 | 𝑊𝑑𝑎𝑡𝑎                             |                | 𝑊𝑑𝑎𝑡𝑎              |     |
| ------------------------- | --------------- | --------------------------------- | -------------- | ------------------ | --- |
|                           |                 |                                   | 𝑚𝑠𝑡            |                    | 𝑚𝑠𝑡 |
|                           | 𝑇𝑜𝑡𝑎𝑙_𝑏𝑦𝑡𝑒𝑠     | = (𝐿𝑒𝑛+1)×                        | −(𝐴𝑑𝑑𝑟𝑒𝑠𝑠 𝑚𝑜𝑑  |                    | )   |
|                           |                 |                                   | 8              |                    | 8   |
| Lane，全局地址的切片，指示当前事务的起始地址在 |                 |                                   | DATA 通道的       | LANE 位置，对齐到网络最大数据位 |     |
| 宽的 clog2                  | 值；Total_bytes，从 | Lane 作为起始地址开始的总有效字节数。这两者统一决定，当前事务 |                |                    |     |
的有效数据流范围：
[𝐿𝑎𝑛𝑒,𝐿𝑎𝑛𝑒+𝑇𝑜𝑡𝑎𝑙_𝑏𝑦𝑡𝑒𝑠−1]
两个信号随 REQ_W 通道传至 Target NIU，由于 Local_Address 是必传信号，Lane 可直接从该信号切
片。RSP_RD 通道也携带响应数据，Lane 和 Total_bytes 必须在 Target NIU 的上下文记录，并随响应
回环至 Initiator NIU。若REQ_W、RSP_RD通路存在 Unify-LINK组件，必须将两个事务定位标信号输
入。

Lane、Total_bytes的共同作用，使得每次位宽转换，有效数据的边界都严格按照转换节点的数据位宽对
| 齐，且剔除部分冗余操作地址。相比紧凑型的数据流，TNIU |     |     |     |     |     |     | 无需记录转换历史和重排序。规格如下，其 |     |     |     |
| ---------------------------- | --- | --- | --- | --- | --- | --- | ------------------- | --- | --- | --- |
中A为输入字节地址，W为下一级链路的字节位宽，tb为有效字节数
| 规则    | 定义      |     |                  |     |         | 说明                  |     |     |     |     |
| ----- | ------- | --- | ---------------- | --- | ------- | ------------------- | --- | --- | --- | --- |
| R1落位  | 字节地址    |     | A 恒落在 beat[A/W]的 |     | Lane(A  | 位置是地址的纯函数，与转换历史、与事务 |     |     |     |     |
|       | mod W)  |     |                  |     |         | 起点都无关               |     |     |     |     |
R2 拍集合  传输的拍为 k ∈ [addr/W⌋, ⌊(addr+tb− 即与有效区间 [addr, addr+tb) 相交的那些
|     | 1)/W⌋]  |     |     |     |     | 拍；整拍不相交者就地剔除  |     |     |     |     |
| --- | ------- | --- | --- | --- | --- | ------------- | --- | --- | --- | --- |
N(W) = ⌊(addr+tb−1)/W⌋ − ⌊addr/W⌋ +
| R3 拍数  |     |     |     |     |     | 每一级的拍数恒等于该位宽下的理论最小值  |     |     |     |     |
| ------ | --- | --- | --- | --- | --- | -------------------- | --- | --- | --- | --- |
1
R4 有效性  beat k 的 lane j 有效 ⟺ addr ≤ k·W+j <  接收端只凭 (addr, tb, k) 即可重建 byte
|     | addr+tb  |     |     |     |     | mask，无需随行  |     |     |     |     |
| --- | -------- | --- | --- | --- | --- | ---------- | --- | --- | --- | --- |

示例图：
Transaction
事务起始相位固定
Externel: Address=0x17, Len=1
Internel: Lane=0x7,Total_bytes=5
|                      |     |     |          |     | beat0 |       |       | beat1 |     |          |
| -------------------- | --- | --- | -------- | --- | ----- | ----- | ----- | ----- | --- | -------- |
|                      |     |     |          |     |       |       |       |       | 1C  | 1D 1E 1F |
| 源(Data Width=4Bytes) |     | 10  | 11 12 13 | 14  | 15    | 16 17 | 18 19 | 1A    | 1B  |          |
|                      |     |     |          | 剔除  |       | beat0 | beat1 | beat2 |     |          |
转换节点
|     |     | 10  | 11 12 13 | 14  | 15  | 16 17 | 18 19 | 1A  | 1B 1C | 1D 1E 1F |
| --- | --- | --- | -------- | --- | --- | ----- | ----- | --- | ----- | -------- |
(Data Width=2Bytes)
|      |     |     | 扩展       | beat0 |     |       |       |     | beat1 | 扩展       |
| ---- | --- | --- | -------- | ----- | --- | ----- | ----- | --- | ----- | -------- |
| 转换节点 |     | 10  | 11 12 13 | 14    | 15  | 16 17 | 18 19 | 1A  | 1B 1C | 1D 1E 1F |
(Data Width=8Bytes)
|     |     |     | 该Wdata下不会涉及到的地址   |     |     | 该Wdata涉及地址且有效数据                   |     |     |     |     |
| --- | --- | --- | ----------------- | --- | --- | --------------------------------- | --- | --- | --- | --- |
|     |     |     | 该Wdata下涉及地址但无有效数据 |     |     | 该Wdata涉及地址（小位宽转大位宽带来的必要开销）且但无有效数据 |     |     |     |     |

Constraints：首拍非对齐，末拍对齐
| 由于对外接口 | Address、Len |     | 语义，有效数据流范围只支持非对齐起始地址，结束地址只能与 |     |     |     |     |     |     | Initiator  |
| ------ | ----------- | --- | ---------------------------- | --- | --- | --- | --- | --- | --- | ---------- |
NIU数据位宽对齐。因此，若MASTER的感兴趣地址不包括整个末拍，或者，Fabric存在小位宽转大位
宽且小位宽实际数据未能对齐大位宽，经过总线后，SLAVE侧可见“比MASTER的感兴趣地址大”的地
址空间被访问。针对普通读、普通写、原子事务进行分析：
| ⚫  普通读事务：数据通路暂无Read Clear特征的SLAVE，即多读一部分地址空间，不会影响功 |     |     |     |     |     |     |     |     |     |     |
| -------------------------------------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
能。如上图，”4Byte转8Byte”引入多余的操作地址空间“10-13”，“1C-1F”。

⚫ 普通写事务：REQ_W通路存在STRB信号，MATSER对末拍不感兴趣的地址，对应的STRB置
无效即可；正对“小位宽转大位宽”，转换器会自动将扩展的STRB置无效，无需MASTER干
涉。”4Byte转8Byte”引入多余的操作地址空间“10-13”，“1C-1F”，但是该地址下的
STRB全部无效；其中“14-16”跟随MASTER的事务Pattern（也是STRB无效）。
⚫ 原子操作：复用REQ_W通路。重点关注，MASTER为大位宽同时SLAVE为小位宽的情况，如
128Byte MASTER发送对齐地址的single原子事务至64Byte SLAVE，即使STRB置位，仍操
作地址为128Byte空间，SLAVE必然发出burst原子事务。然而目前L2 Cache不支持Burst
Atomic，因此有Atomic需求的MASTER，其位宽必须小于等于SLAVE位宽。
若要支持MASTER、SLAVE的数据流范围完全一致，则需要增加额外信号Addition控制：
[𝐿𝑎𝑛𝑒,𝐿𝑎𝑛𝑒+𝑇𝑜𝑡𝑎𝑙_𝑏𝑦𝑡𝑒𝑠−𝐴𝑑𝑑𝑖𝑡𝑖𝑜𝑛−1]
经初步商讨，暂时不加，有MASTER IP侧适配。

| 1.2 事务级的  | Switch 仲裁路由粒度                          |     |     |      |
| --------- | -------------------------------------- | --- | --- | ---- |
| 网络型总线一般采用 | Flit 级作为最小路由单元，但是并不是仲裁粒度。仲裁粒度有事务级、Filt |     |     | 级，对比 |
如下
|      | 事务级                     |                 | Flit级                |      |
| ---- | ----------------------- | --------------- | -------------------- | ---- |
| 路由与仲 | Switch以一个完整事务包作为仲裁粒度，一旦 |                 | Switch 不感知事务，按照每一个拍的 | Flit |
| 裁规则  | 某个出口被授权于指定入口的事务，必须要等    |                 | 单元为仲裁粒度，同一个出口呈现出不同   |      |
|      | 到这个事务传输完成，直到            | Last Assertion， | 事务交织传递               |      |
才释放占用
| 乱序与重 | 同一个事务按照顺序完整路由，端侧无需设置 |     | 不同事务穿插到达，乱序由总线自身引 |     |
| ---- | -------------------- | --- | ----------------- | --- |
| 组    | Buffer重组，面积时序开销小     |     | 入，若端侧不支持读写交织，则需要  | ROB |
对事务重组，事务完整后才能下发，引入
大量面积开销和时序瓶颈
| 仲裁频次  | 事务级只需要头拍仲裁、末拍释放，无需每拍 |     | 每拍Flit都需要仲裁，翻转相对多  |     |
| ----- | -------------------- | --- | ------------------ | --- |
仲裁，翻转少
| HOL 阻 | 事务级仲裁，若事务反压则会一直占用输出， |     | Flit级路由，没有队列，阻塞相对较小  |     |
| ----- | -------------------- | --- | -------------------- | --- |
| 塞     | HOL相对严重              |     |                      |     |

结合项目经验，LITEBUS采用事务级仲裁，原因如下：
⚫  LITEBUS初衷之一为轻量化，做时序boost、面积缩减。Flit引入的ROB与BASIC CORE相悖。
⚫  事务级仲裁在低功耗方面有天然优势。
⚫  至于HOL阻塞，可在Switch部署适量XBAR FIFO，也极大缓解。最新实验表明，16x16
XBAR_L1 两级总线在8深度下的FIFO，Switch的出口空转率可压低至约10%。
⚫  事务级仲裁是比较成熟技术，FlexNoC就是运用该技术之一的商业IP。Flit级暂未接触，研究成本
和风险不可控。

| 1.3 读响应交织的       | Fragment | 规划          |         |                |
| ---------------- | -------- | ----------- | ------- | -------------- |
| 通常情况下，SLAVE IP（如 | DDRC）支持多 | Outstanding | 事务处理，由于 | DDR 颗粒限制，读操作下的 |
数据并不是一定按照顺序、或者单事务准备完成，存在“部分读请求的部分数据已经准备，可以发射”
| 的情况，若等完整包重整完整，会 |     | STALL 读数据的放射，降低效率，因此读响应交织应运而生。它使得 |     |     |
| --------------- | --- | --------------------------------- | --- | --- |
SLAVE IP在“同ID保序”前提下，自由排布多个事务的读数据穿插回复，以提高效率。
|     |     |     |     |     |
| --- | --- | --- | --- | --- |
LITEBUS为性能需求，必须支持该Feature，但是与1.2节描述的“事务级仲裁Switch”原则有些冲突，
因此在 Target NIU 做适配，以兼容目前的 Switch 技术。Target NIU 将交织返回的每一拍，转化为独
立的“事务”，带有独立的Lane、Total_bytes、Last等特征信号，使得“事务”正常通过Switch。

此外，若 SLAVE 侧支持读交织，同时 SLAVE 侧数据位宽小于 MASTER 侧，那么 Target NIU 不能将不
足MASTER一拍数据的small fragment直接返回MASTER，否则MASTER会将这small fragment当
成完整一拍数据（large fragment）返回，造成功能 failure。因此，Target NIU 必须部署 Assembly
Buffer，将small fragment拼成large fragment后发射。考虑到极端情况，每一笔Pendingtrans都需
要同时重组， Assembly Buffer必须为MAX Pendingtrans深度，Master MAX Wdata宽度的规格。

Recommandatiom、Constraints：少用Assembly Buffer
Assembly Buffer同样带来相对大的面积、时序开销，因此建议：
⚫  有读交织需求的PAIR（MASTER->SLAVE），MASTER的数据位宽最好小于等于SLAVE。
⚫  SLAVE Pendingtrans尽可能设置小。

MASTER IP
lg_frag0
lg_frag1
EXT_RSP_RD
INIU
Master Wdata = N*Slave Wdata
INT_RSP_RD
lg_frag0
lg_frag1
Unify LINK
sm_frag0_0 sm_frag0_1 sm_frag0_N-1
old sm_frag0_0
sm_frag0_1
Large fragment
sm_frag0_N-1
乱序重组
sm_frag1_0
INT_RSP_RD sm_frag1_1
Large fragment
new sm_frag1_N-1
TNIU
slave Wdata
Assembly Buffer
0 sm_frag0_0 sm_frag0_1 sm_frag0_N-1
1 sm_frag1_0 sm_frag1_1 sm_frag1_N-1
MAX(PNT)-1
master Wdata
old sm_frag0_0
sm_frag1_0
EXT_RSP_RD
交织乱序 sm_frag1_1
sm_frag0_1
new
SLAVE IP

1.4 事务 ID 变迁
LITEBUS使用三种ID实现事务、Initiator NIU、Target NIU的身份证明，用于事务匹配与认证、路由寻
址与事务生命周期管理。
⚫ Transaction ID：跟随具体Transaction的身份认证信息，BASIC-CORE不允许网络中同时存在多
笔同ID事务，因此，同MASTER的多笔在途Transaction都有独一无二的ID。
◼ Extid（MASTER）：MASTER IP发送的原生事务ID。读读事务、写写事务、普通事务和原子
事务之间，不能存在同ID。
◼ Extid（SLAVE）：CMD TABLE的索引ID，即Local ID，其中该ID对于SLAVE IP而言，多
笔在途Transaction也是有独一无二的ID。
◼ Intid（Fabric）：网络内部ID，多个MASETR的ID位宽可能不一致，需要同步到最大位宽
（使用Padding0补齐）。
⚫ SrcID：每个Initiator NIU有唯一的ID标识，用于响应通路上的路由，Switch必须输入该信号，
查询内置路由表，做互联节点传递。
⚫ DestID：每个Target NIU有唯一的ID标识，用于请求通路上的路由，Switch必须输入该信号，
查询内置路由表，做互联节点传递。
Constraints：不支持同ID传输
CMD TABLE 为每一个新来的事务分配 ID，如果 CMD TABLE 没有可用槽，达到 Pendingtrans 上限，
则需要反压。CMD TABLE 只记录 Srcid、IntID，不会记录同一个 PAIR(Srcid、IntID）的到达顺序，因
此不支持同ID传输。

MASTER IP
Address(global) extid(local)
| Initiator NIU |     | 地址译码获得Destid |     | 统一到网络中ID最大位宽 |
| ------------- | --- | ------------ | --- | ------------ |
Srcid注入
|     | Srcid | Destid | Padding 0 extid(local) |     |
| --- | ----- | ------ | ---------------------- | --- |
intid(global)
直通
| Fabric | 直通  |     |     |     |
| ------ | --- | --- | --- | --- |
输入Switch路由表Routing 直通
直通
Srcid Intid(global)
丢弃，不再使用
|            | 注册进CMD表 |           | 注册进CMD表 |     |
| ---------- | ------- | --------- | ------- | --- |
| Target NIU |         | LID Srcid | Intid   |     |
0
|     | CMD  | 1   |     |     |
| --- | ---- | --- | --- | --- |
Table

MAX(PNT)-1
| SLAVE IP | Extid(SLAVE) |     |     |     |
| -------- | ------------ | --- | --- | --- |
索引查CMD表
|     |     | LID Srcid | Intid   |     |
| --- | --- | --------- | ------- | --- |
0
1

Target NIU
查出Srcid、Intid
|     |     | Srcid | Intid(global) |     |
| --- | --- | ----- | ------------- | --- |
|     |     | 直通    | 直通            |     |
Fabric
输入Switch路由表Routing
intid(global)
|     |     | Srcid | Padding 0 extid(local) |     |
| --- | --- | ----- | ---------------------- | --- |
Initiator NIU
|     |     | 丢弃，不再使用 | 截位与MASTER IP一致 |     |
| --- | --- | ------- | -------------- | --- |
extid(local)
MASTER IP

1.5 事务拆分规划
BASIC-CORE不支持任何自主拆包行为。
根据 GP3A 项目经验，交织粒度与 L2  Cache 一致，256Byte（L2  Cache，maxlen=4 拍，
| wdata=512bit）。交织器位于 |     | MATSER | 和总线之间，事务准入的绝对前提是当前事务地址范围不可跨 |     |     |     |
| ------------------- | --- | ------ | --------------------------- | --- | --- | --- |
越256Byte边界，因此准入总线的事务天然不会超过L2 Cache的MAX Burst Size。受到交织粒度的限
制，若MASTER意图发出超过SLAVE侧Size上限，MASTER需完成拆包流程。
| 基于该原因，拆包并不属于 |     | BASIC-CORE | 的必须选项，且拆包后的子事务为实现保序功能，多笔子事务 |     |     |     |
| ------------ | --- | ---------- | --------------------------- | --- | --- | --- |
会同时继承母事务ID，进入“同ID保序”、“事务上下文记录”问题。若要处理（如交织粒度放宽），
配合EXTENDED-CORE中的One-trans-fly或者ROB处理较为妥善（Master侧）。
若SLAVE IP支持同ID处理，一种不依赖One-trans-fly或者ROB的Simple拆包组件更加轻量化。只
需要在请求侧TNIU出口，保持wdata切分len，划分子事务请求；并在响应侧处理子事务带来的多笔响
应问题即可（然而，L2 Cache并不支持同ID处理）。
Constraints：MASTER的MAX Burst Size(（len+1）*wdata) 不能超过SLAVE的，若超过，
MASTER必须自行拆包处理。
|     | MST0              | MST1                | MSTn-1                |      |      |      |
| --- | ----------------- | ------------------- | --------------------- | ---- | ---- | ---- |
|     | wdatam0*(lenm0+1) | wdatam1*(lenm1+1)   | wdatamn-1*(lenmn-1+1) |      |      |      |
|     | INIU              | INIU                | INIU                  | INIU | INIU | INIU |
B A S I C- C O RE ： M A S T E R 不 可 发 送大 BASIC-CORE all-connect BASIC-CORE
| 于 S L A V E  M A X B u r st   Si z e的 事 务 |                       |                      |                             |      |      |      |
| ----------------------------------------- | --------------------- | -------------------- | --------------------------- | ---- | ---- | ---- |
|                                           | TNIU                  | TNIU                 | TNIU                        | TNIU | TNIU | TNIU |
|                                           |                       | wdata s1*(lens1+1)   | S L V                       |      |      |      |
|                                           | S L V 0               |                      | wdat a m * - ( 1 lensn-1+1) |      |      |      |
|                                           | wdata s 0* ( lens0+1) |                      | sn -1                       |      |      |      |
beat4
|     |     | b e a t 2 |     |     | old beat0 b e a t 1 |     |
| --- | --- | --------- | --- | --- | ------------------- | --- |
CONTE X T  TABLE b e a t 2 Transaction 1：len=4, id=0x1 Transaction(RD) 1：id=0x1 WR 1_1 b e a t 2 Transaction(RD) 1：id=0x1
|                     |     | b e a t 1     |     |     | b e a t 3       |     |
| ------------------- | --- | ------------- | --- | --- | --------------- | --- |
| L I D Len or subnum |     | old b e a t 0 |     |     | b e a t 4 rlast |     |
0
| 1                   |     | Simple Split (Req) eg: lens1最大值为7，  lenrls1最大值为3 |     |     |                     |     |
| ------------------- | --- | ------------------------------------------------ | --- | --- | ------------------- | --- |
|                     |     |                                                  |     |     | Simple Split(Rsp)   |     |
|                     |     | Sub-Transaction 1_1：len=0, id=0x1                |     |     |                     |     |
| M A X ( P N T ) - 1 |     | beat4                                            |     |     | old beat0 b e a t 1 |     |
b e a t 3 S u b - T r a n s a c t i o n ( W R )   1 _ 0 ： i d = 0 x 1 W R   1 _0 old b e a t 2 S u b - T r a n s a c t i o n ( R D )   1 _ 0 ：   i d = 0 x 1
记 录 每 笔 母 读 事 务 的 L e n ， 用 于 处 理 RS P_ RD 的 非 尾 b b e e a a t t 2 1 Sub-Transaction 1_0：len=3, id=0x1 S u b - T r a n s a c t i o n ( W R )   1 _ 1 ： i d = 0 x 1 WR   1 _ 1 b e a t 3 r l a s t
b e at的   r la s t  a s s er t i o n   b e a t 0 S u b - T r a n s a c t i o n ( R D )   1 _ 1 ：   i d = 0 x 1
记 录 每 笔 母 写 事 务 的 拆 分 笔 数 ， 用 于 处 理 RS P_ W R的 b e a t 4 r l a s t
| 非末子事务的写响应 |     | Real SLV1 |     |     | Real SLV1   |     |
| --------- | --- | --------- | --- | --- | ----------- | --- |
wdatas1*(lenrls1+1) SLAVE支持同ID
|     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- |

| 1.6 QoS 规划  |     |     |     |     |     |     |
| ----------- | --- | --- | --- | --- | --- | --- |
FlexNoC 采用“Urgency、Pressure、Hurry”较为复杂的全局 QoS 复杂机制，LITEBUS 本着轻量化原
则，只实现面效高的局部QoS。
⚫  支持MASTER IP按照事务粒度赋予QoS信号
⚫  QoS（0~4bit），数值越高，优先级越高
⚫  Switch仲裁是QoS信号的唯一消费者，做局部QoS，仅在Switch仲裁器作用
⚫  Switch 仲裁器增加Round-Robin-QoS机制，仲裁优先级信号为FIFO的最大有效优先级
⚫  MASTER、SLAVE只有请求通道侧存在QoS信号，响应通道则没有，需要在TNIU中的CMD
TABLE按照Pendingtrans锁存，并响应返回时查表并带入RSP_WR和RSP_RD，输入至链路
上的Switch。
IN1
IN0
Switch
FIFO
FIFO
| 索引 Payload   | QoS Vld |      |     |            |         |      |
| ------------ | ------- | ---- | --- | ---------- | ------- | ---- |
|              |         |      |     | 索引 Payload | QoS Vld | FIFO |
| 0            | 0 1     |      |     |            |         |      |
| 1            | 3 1     |      |     | 0          | 0 1     |      |
| 2            | 1 1     | FIFO |     | 1          | 2 1     |      |
|              |         |      |     | 2          | 0 0     |      |
|              |         |      |     |            |         |      |
|              |         |      |     |            |         |      |
| FIFO_DEPTH-1 | 0 0     |      |     |            |         |      |
0 0
|     |     | Arbiter |     |     | Arbiter |     |
| --- | --- | ------- | --- | --- | ------- | --- |
事务输出顺序

0 1 0 1 2
|     |     | OUT0 |     |     | OUT1 |     |
| --- | --- | ---- | --- | --- | ---- | --- |

| Constraints：QoS                           | 的生产者是 | MASTER，MASTER       | 要慎用 | QoS 信号，推荐仅对紧急事务施加高 |     |       |
| ----------------------------------------- | ----- | ------------------- | --- | ------------------ | --- | ----- |
| QoS，总线会优先仲裁该事务，减少该事务的请求响应延时，不推荐将大部分事务都设置高 |       |                     |     |                    |     | QoS。  |
| 优先级相同情况下，仍然会走                             |       | Round-Robin，即“把所有事务 |     | Qos设置成0，和设置成       |     | 15”，不 |
会有任何性能收益。
|     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- |

1.7 Pendingtrans 限制（Outstanding）
FlexNoC为“同ID保序”，需记录事务上下文。然而在硬件资源视角上，上下文不可能无限大，必须指
定一个具体数值，指示可用槽数量，即一般是 MASTER/SLAVE 的 MAX Pendingtrans。在一些面积、
时序权衡下，NIU的Pendingtrans可能小于MASTER/SLAVE的MAX Pendingtrans，因此在Initiator
NIU和Target NIU处会引入Pendingtrans闸门。
BASIC-CORE将记录上下文操作限制在Target NIU。Initiator NIU则无任何Pendingtrans反压。
Constraints：Pendingtrans决定CMD TABLE的深度、查表与归并逻辑级数。Pendingtrans越
大，伴随着更大的面积、时序开销，需要谨慎评估。
1.8 Farbic 准入前的信号位宽统一
TBD