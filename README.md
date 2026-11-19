# 网格转 B-rep：OCCT 首日原型技术规格

本仓库保存一份可交给 Devin Bot 执行的工程任务规格。

**状态：设计与验收要求；尚无转换器实现、编译结果或测试通过记录。**

首日目标是证明从平面多面体网格到共享拓扑 B-rep 的转换链路。复杂曲面重建及复杂倒角属于后续工作。本规格不保证任意网格可还原为原始 CAD，也不代表任何促销资格或验证结果。

## 任务目标

在独立的 `native/mesh_brep/` 目录实现 C++ 原型，使用 CMake 构建，记录实际使用的 OCCT 版本。若接入现有仓库，先阅读仓库约定，并保持原有功能正常。

处理链路：

```text
索引三角网格
→ 输入检查、单位转换和局部坐标
→ 平面面片与特征边链邻接图
→ 支撑平面求交、联合求解角点
→ VertexCache、EdgeCache
→ pcurve、Wire、Face
→ 闭合 Shell、Solid
→ 独立几何审计、OCCT 有效性检查、BREP 导出
```

首日支持闭合、可定向、无自交的平面多面体网格，包括凸体和具有凹角的简单多面体。开放网格、非流形输入、周期接缝、退化边和需要非平面支撑曲面的区域返回明确错误或“不支持”。

## 1. 输入及误差预算

输入使用 JSON：

```json
{
  "unit": "mm",
  "vertices": [[0, 0, 0], [100, 0, 0], [0, 100, 0]],
  "triangles": [[0, 1, 2]]
}
```

该示例仅说明格式，只有一个三角形，应被闭合性检查拒绝。

保留原始顶点和三角形 ID。内部统一毫米；平移到包围盒中心附近，保存逆变换。初版按输入索引建立连接关系，不自动焊接空间近邻顶点。

分别配置下列参数，不使用一个 epsilon 覆盖全部误差：

| 参数 | 含义 |
| --- | --- |
| `meshFitTolerance` | 重建支撑几何与输入网格的允许偏差 |
| `topologyTolerance` | 新建拓扑的目标几何容差 |
| `toleranceCap` | 操作结果允许的存储容差硬上限 |
| `maxNodeDisplacement` | 重建角点相对输入位置的最大移动量 |
| `angularTolerance` | 平行、共面等判断使用的角度容差 |

长度单位为毫米，角度单位为弧度。检查数值有限、为正；目标容差应符合所用 OCCT API 的有效精度下限，且不超过硬上限。为测试模型提供明确配置并解释取值。失败重试不得自动放大配置。

网格重建误差和 B-rep 内部一致性误差分别审计。网格允许 0.01 mm 拟合偏差，不表示应把所有顶点容差设置为 0.01 mm。

## 2. 面片和边链

建立半边邻接结构，检查退化三角形、重复面、方向一致性、网格边关联面数量、顶点 link 的流形条件，以及非相邻三角形异常相交。相邻元素的相交检查必须正确排除合法共享顶点和共享边。

合并满足平面拟合条件的邻接三角形。每次合并后检查整个候选面片所有顶点到支撑平面的最大距离，避免仅凭邻接法线相近而逐步漂移。首日平面三角形分支可利用线性距离性质给出面内偏差界；对其他几何不得声称仅检查顶点就得到全域误差界。

提取面片间公共边链，保存：

```text
BoundaryId
两侧 PatchId
有序原始网格边及采样点
起终 NodeId
在两侧面片中的遍历方向
```

在交汇点、方向突变或邻接面片对变化处拆分。保留原始网格来源映射。重复、分叉或无法构成简单边界的情况必须返回诊断。

## 3. 支撑曲面求交

实现接口，其具体 C++ 类型可依项目约定调整：

```cpp
IntersectionResult intersectSupports(
    const Patch& a,
    const Patch& b,
    const BoundaryChain& evidence,
    const ErrorBudget& budget);
```

首日流程：

1. 从已验证面片取得 `gp_Pln`，使用 `IntAna_QuadQuadGeo` 求平面交线。
2. 检查 `IsDone()`、`TypeInter()` 和解的数量；只有预期结果才继续。
3. 共面情形返回面片合并阶段；平行但不重合返回错误。近乎平行且数值不稳定时明确拒绝。
4. 用原始边链验证交线，记录最大偏差，并检查它对应相邻面片的实际边界。
5. 交线是无限直线；待角点最终确定后再取得有限端点参数。不能把无限交线直接作为已裁剪边。

保留一般曲面求交接口供后续扩展。未来接入 `GeomAPI_IntSS` 时必须检查计算状态，遍历交线候选，依据边链证据和面片有效域选择，必要时拆分；候选不唯一时拒绝猜测。支撑曲面求交不等于已经完成面域裁剪。

## 4. VertexCache

建议数据模型：

```cpp
struct VertexKey {
    ComponentId component;
    NodeId node;
};

struct VertexRecord {
    gp_Pnt position;
    TopoDS_Vertex vertex;
    std::vector<BoundaryId> incidentBoundaries;
    double maxEndpointResidual;
};
```

`NodeId` 来自边链邻接图。坐标相同或接近不能直接决定拓扑身份。不同连通分量及薄壁两侧不合并。

在创建 OCCT 顶点前：

1. 收集该节点关联的所有支撑平面和交线。
2. 用具有秩检测的求解器联合求解角点，记录条件性和残差。可以使用 SVD，但不能把超预算最小二乘解当作有效公共交点。
3. 欠约束、关联约束不相容、节点移动量超预算时返回错误。
4. 检查角点到每条关联交线的距离满足目标容差。
5. 每个 `VertexKey` 只创建一个 `TopoDS_Vertex`，关联边复用该顶点。

位置确定后冻结。任何位置改变都必须重建受影响的候选拓扑，并重新检查全部关联边。禁止用增大顶点容差代替角点求解。

至少验证：

```text
max_e distance(C_e(u_endpoint), P_vertex)
    <= storedVertexTolerance
    <= toleranceCap
```

## 5. EdgeCache

```cpp
struct EdgeKey {
    ComponentId component;
    PatchId patchLow;
    PatchId patchHigh;
    BoundaryId boundary;
    SegmentId segment;
};

struct EdgeRecord {
    Handle(Geom_Curve) curve;
    double firstParameter;
    double lastParameter;
    VertexKey start;
    VertexKey end;
    TopoDS_Edge edge;
};
```

键必须区分同一对面片之间的多条独立边界。为键实现明确的相等与哈希逻辑。

构造顺序：

1. 将最终共享角点投影到交线，取得端点参数。
2. 检查参数有限、范围有效、边长非零及端点残差。
3. 按统一参数方向构建一次 `BRepBuilderAPI_MakeEdge(curve, startVertex, endVertex, u0, u1)`，检查构造状态及错误码。
4. 缓存保存规范方向的 Edge。各面按其有序边界取得 Edge 或反向使用形式，保持底层拓扑共享。
5. 通过 `IsSame()` 及实际顶点身份验证相邻面引用共享实体。不要每个面独立创建一条重合边再依靠缝合合并。

所有缓存只属于当前候选模型。任一步骤失败时销毁候选。普通 `TopoDS_Shape` 赋值会共享底层数据，不能作为事务回滚快照。

## 6. pcurve 参数化

首日对直线与平面使用解析映射。设：

```text
C(t) = O + tD
S(u,v) = P + uX + vY
```

其中 `X`、`Y` 是平面正交单位基，D 是三维曲线实际参数导数。则：

```text
u(t) = (O-P)·X + t(D·X)
v(t) = (O-P)·Y + t(D·Y)
```

保留同一个参数 t。二维方向的长度必须显式保留；不能归一化方向后丢失参数缩放。选择能保留参数化的二维曲线表示，并设置与三维边一致的有限范围。

用 `BRep_Builder::UpdateEdge` 为同一条 Edge 添加两侧支撑曲面上的 pcurve。使用的曲面对象及 Location 必须与最终 Face 一致。

对两个关联面分别验证：

```text
distance(C_e(t), S_f(pcurve_e_f(t))) <= storedEdgeTolerance
storedEdgeTolerance <= toleranceCap
```

不能只设置 `SameParameter` 或 `SameRange` 标志而不验证几何。若使用 `BRepLib::SameParameter`，处理后重新审计残差、容差和实体身份。

## 7. Wire、Face、Shell 和 Solid

在每个面片的 UV 空间识别外环与孔洞，检查闭合、方向、交叉和包含关系。首日仅接受可明确分类的简单面域。

按有序边界组装 Wire 和 Face，并直接复用缓存共享拓扑组装 Shell。验证闭合性、流形性、朝向及实体有效性后才构造或接受 Solid。构造 API 返回成功不是闭合有效实体的充分证据。

首日不依赖大容差 sewing 修补不相容几何。如后续启用 sewing，须限制其参数、检查输入容差叠加行为并审计实际输出。任何 ShapeFix 操作也受相同预算约束。

## 8. 审计和失败语义

在 Edge、Face、Solid 构造后分别记录：

- 顶点到关联边端点的实际偏差。
- 三维边与每个 pcurve 映射结果的实际偏差。
- Vertex、Edge、Face 的存储容差以及操作前后增长量。
- 共享拓扑身份、边的关联面数量、顶点邻域、面环和壳闭合性。
- 重建结果与输入网格的偏差以及自交检查结果。

平面/直线分支尽量使用解析计算或有依据的误差界。采用采样的检查必须报告采样方式和覆盖范围，不能将采样结果称为全局误差证明。网格拟合残差不能替代实体内部一致性检查，反之亦然。

运行 `BRepCheck_Analyzer`，同时保留上述独立审计。任何实际偏差超过允许值、容差超过硬上限或拓扑检查失败，均拒绝提交。不要通过强制调小存储容差数值掩盖实际几何间隙。

错误示例（数值仅说明输出格式）：

```json
{
  "stage": "vertex_solve",
  "code": "ENDPOINT_RESIDUAL_EXCEEDED",
  "nodeId": 17,
  "boundaryIds": [4, 8, 12],
  "measured": 0.002,
  "allowed": 0.000001,
  "unit": "mm"
}
```

所有异常必须保留阶段、输入来源 ID、测量值及阈值。错误输入不能导出伪装为成功的 Solid。

## 9. 验收样例与交付

必须自动生成并执行以下真实测试：

| 输入 | 预期 |
| --- | --- |
| 三角化立方体 | 恢复 8 个共享顶点、12 条共享边、6 个平面面，得到有效闭合实体 |
| 简单凹多面体 | 正确处理凹边、面环及朝向 |
| 孔洞边界样例 | 正确区分外环和内环；无法支持时明确报告范围限制 |
| 间距很近的两个独立实体 | 身份保持分离，不因坐标接近而合并 |
| 坐标重复但索引未连接 | 根据连接关系报告问题，不自动焊接 |
| 非流形、退化或超拟合预算输入 | 给出明确错误 |
| 近乎平行支撑面 | 稳定求解或明确拒绝，不产生远处异常角点 |
| 调换三角形输入顺序 | 几何和拓扑验收结果一致；不要求来源 ID 完全相同 |

立方体样例使用解析可检查的边、端点和 pcurve 残差作为首个成功标准。测试应检查真实共享对象身份和失败行为，不只检查函数是否返回对象。

交付源码、CMake、实际 OCCT 版本与依赖说明、输入生成脚本、成功样例的 `.brep`、逐阶段 `audit.json`、可复现构建/测试命令、实际执行结果及支持范围。

未完成的实现、编译或测试必须如实标注，并提交具体阻塞证据。本规格本身不构成测试通过证明。

## 官方 API 参考

- [IntAna_QuadQuadGeo：平面和二次曲面求交](https://occt3d.com/dev/doc/refman/html/class_int_ana___quad_quad_geo.html)
- [GeomAPI_IntSS：一般支撑曲面求交](https://occt3d.com/dev/doc/refman/html/class_geom_a_p_i___int_s_s.html)
- [BRepBuilderAPI_MakeEdge：曲线、共享顶点和参数](https://occt3d.com/dev/doc/refman/html/class_b_rep_builder_a_p_i___make_edge.html)
- [BRep_Builder：添加 pcurve 和几何表示](https://www.occt3d.com/dev/doc/refman/html/class_b_rep___builder.html)
- [BRepBuilderAPI_MakeFace：支撑曲面和边界](https://occt3d.com/dev/doc/refman/html/class_b_rep_builder_a_p_i___make_face.html)
- [BRepLib：SameParameter 等操作](https://occt3d.com/dev/doc/refman/html/class_b_rep_lib.html)
- [BRepCheck_Analyzer：有效性检查](https://occt3d.com/dev/doc/refman/html/class_b_rep_check___analyzer.html)
- [BRepBuilderAPI_Sewing：缝合参数及局部容差](https://www.occt3d.com/dev/doc/refman/html/class_b_rep_builder_a_p_i___sewing.html)

API 签名及行为以实际编译所用 OCCT 版本为准。
