# 语义导航核心算法快速参考

## 1. Fast-Slow双进程架构

### Fast Path (System 1) - 无需LLM
```python
def fast_resolve(instruction: str, scene_graph: dict) -> Optional[GoalResult]:
    """
    快速路径：场景图直接匹配，~10ms响应
    适用于70%+的简单导航场景
    """
    # 1. 提取关键词
    keywords = extract_keywords(instruction)  # "红色灭火器" → ["红色", "灭火器"]

    # 2. 场景图匹配
    candidates = []
    for obj in scene_graph['objects']:
        # 2.1 标签匹配
        label_score = fuzzy_match(keywords, obj['label'])

        # 2.2 CLIP相似度（视觉-语言）
        clip_score = clip_similarity(instruction, obj['clip_feature'])

        # 2.3 检测器置信度（加权观测次数）
        detector_score = obj['score'] * min(obj['detection_count'] / 10, 1.0)

        # 2.4 空间关系提示
        spatial_score = check_spatial_hints(instruction, obj, scene_graph)

        # 多源融合
        fused_score = (0.35 * label_score +
                      0.35 * clip_score +
                      0.15 * detector_score +
                      0.15 * spatial_score)

        candidates.append((obj, fused_score))

    # 3. 选择最高分
    best_obj, best_score = max(candidates, key=lambda x: x[1])

    # 4. 置信度判断
    if best_score > 0.75:  # 高置信度 → Fast Path成功
        return GoalResult(
            action="navigate",
            position=best_obj['position'],
            confidence=best_score,
            path="fast"
        )
    else:
        return None  # 进入Slow Path
```

### Slow Path (System 2) - 调用LLM
```python
def slow_resolve(instruction: str, scene_graph: dict) -> GoalResult:
    """
    慢速路径：ESCA过滤 + LLM推理，~2s响应
    适用于复杂场景和低置信度情况
    """
    # 1. ESCA选择性Grounding（关键创新）
    filtered_graph = esca_filter(instruction, scene_graph)
    # 200物体 → 15物体，tokens减少90%

    # 2. 构建Prompt
    prompt = f"""
    指令: {instruction}
    场景图: {json.dumps(filtered_graph, ensure_ascii=False)}

    请分析场景图，返回目标物体ID或建议的动作。
    """

    # 3. LLM推理（多后端容错）
    response = llm_client.chat(prompt)

    # 4. 解析响应
    result = parse_llm_response(response)

    # 5. 可选：视觉验证（置信度<0.5时）
    if result.confidence < 0.5:
        image = capture_current_view()
        vision_result = gpt4o_vision.verify(instruction, image)
        result.confidence = vision_result.confidence

    return result
```

## 2. ESCA选择性Grounding

### 核心思想
将大场景图过滤为小子图，减少LLM输入tokens，提升推理精度

### 算法实现
```python
def esca_filter(instruction: str, scene_graph: dict) -> dict:
    """
    ESCA (Efficient Selective Context Aggregation)
    NeurIPS 2025论文方法
    """
    # 1. 关键词提取
    keywords = extract_keywords(instruction)
    # "去厨房找红色杯子" → ["厨房", "红色", "杯子"]

    # 2. 关键词匹配（初始集合）
    relevant_objects = []
    for obj in scene_graph['objects']:
        if any(kw in obj['label'] or kw in obj.get('attributes', [])
               for kw in keywords):
            relevant_objects.append(obj['id'])
    # 结果: [obj_5: "杯子", obj_12: "厨房门"]

    # 3. 1-hop关系扩展（添加邻居）
    for obj_id in list(relevant_objects):
        for relation in scene_graph['relations']:
            if relation['subject_id'] == obj_id:
                relevant_objects.append(relation['object_id'])
            elif relation['object_id'] == obj_id:
                relevant_objects.append(relation['subject_id'])
    # 结果: 添加 [obj_6: "桌子"（杯子在上面）, obj_13: "走廊"（厨房门旁边）]

    # 4. 区域扩展（同区域物体）
    relevant_regions = set()
    for obj_id in relevant_objects:
        obj = get_object_by_id(scene_graph, obj_id)
        if obj.get('region_id'):
            relevant_regions.add(obj['region_id'])

    for obj in scene_graph['objects']:
        if obj.get('region_id') in relevant_regions:
            relevant_objects.append(obj['id'])
    # 结果: 添加厨房区域的其他物体

    # 5. 构建过滤后的子图
    filtered_graph = {
        'objects': [obj for obj in scene_graph['objects']
                   if obj['id'] in relevant_objects],
        'relations': [rel for rel in scene_graph['relations']
                     if rel['subject_id'] in relevant_objects
                     and rel['object_id'] in relevant_objects],
        'regions': [reg for reg in scene_graph['regions']
                   if reg['region_id'] in relevant_regions]
    }

    return filtered_graph
    # 最终: 200物体 → 15物体（减少92.5%）
```

### 效果
- **Token减少**: 200物体 → 15物体，减少90%+ tokens
- **精度提升**: 开源模型（Qwen-Max）可超越GPT-4
- **成本降低**: API费用大幅下降

## 3. MTU3D Frontier评分

### 核心思想
结合距离、新颖度、语言提示和Grounding Potential，智能选择探索方向

### 算法实现
```python
def score_frontier(frontier: Frontier,
                   instruction: str,
                   scene_graph: dict,
                   topological_memory: TopologicalMemory) -> float:
    """
    MTU3D (ICCV 2025) Frontier评分算法
    """
    # 1. 距离分数（近的优先）
    distance = compute_distance(robot_pose, frontier.position)
    distance_score = 1.0 / (1.0 + distance / 10.0)  # 归一化到[0,1]

    # 2. 新颖度分数（未去过的优先）
    novelty_score = topological_memory.compute_novelty(frontier.direction)
    # 基于8扇区访问密度统计

    # 3. 语言分数（附近有相关物体）
    keywords = extract_keywords(instruction)
    nearby_objects = get_objects_near_frontier(frontier, scene_graph, radius=3.0)
    language_score = 0.0
    for obj in nearby_objects:
        if any(kw in obj['label'] for kw in keywords):
            language_score += 0.3
    language_score = min(language_score, 1.0)

    # 4. Grounding Potential（目标出现概率）- MTU3D核心创新
    grounding_score = compute_grounding_potential(
        frontier, instruction, scene_graph, topological_memory
    )

    # 5. 加权融合
    final_score = (0.2 * distance_score +
                  0.3 * novelty_score +
                  0.2 * language_score +
                  0.3 * grounding_score)

    return final_score


def compute_grounding_potential(frontier, instruction, scene_graph, memory):
    """
    Grounding Potential: 目标在该方向出现的概率
    """
    score = 0.0
    keywords = extract_keywords(instruction)

    # 4.1 空间梯度（方向上有相关物体 → 目标可能更远）
    direction_vector = normalize(frontier.position - robot_pose.position)
    for obj in scene_graph['objects']:
        if any(kw in obj['label'] for kw in keywords):
            obj_direction = normalize(obj['position'] - robot_pose.position)
            alignment = dot(direction_vector, obj_direction)
            if alignment > 0.7:  # 同方向
                score += 0.3

    # 4.2 关系链延伸（目标的关联物体在该方向）
    # 例如: "找灭火器" → "门"和"走廊"在该方向 → 灭火器可能也在
    related_objects = get_related_objects(keywords, commonsense_kb)
    for rel_obj in related_objects:
        if object_in_direction(rel_obj, frontier.direction, scene_graph):
            score += 0.2

    # 4.3 常识共现（知识库）
    # fire_extinguisher ↔ door, stairs, corridor
    cooccur_score = commonsense_kb.query_cooccurrence(keywords, frontier.context)
    score += cooccur_score * 0.2

    return min(score, 1.0)
```

### 效果
- 比VLFM基线提升14-23%
- 探索效率显著提高
- 减少无效探索

## 4. ConceptGraphs增量式场景图

### 核心思想
多帧融合构建一致的3D场景图，支持实例跟踪和空间关系推理

### 算法实现
```python
class InstanceTracker:
    """
    ConceptGraphs (ICRA 2024) 增量式场景图构建
    """
    def __init__(self):
        self.objects = []  # 已跟踪的物体实例
        self.relations = []  # 空间关系
        self.regions = []  # 区域聚类

    def update(self, detections: List[Detection3D], frame_id: int):
        """
        更新场景图（每帧调用）
        """
        for det in detections:
            # 1. 数据关联（匹配已有实例）
            matched_obj = self._associate(det)

            if matched_obj:
                # 2. EMA位置平滑（减少抖动）
                matched_obj.position = (0.3 * det.position +
                                       0.7 * matched_obj.position)
                matched_obj.detection_count += 1
                matched_obj.last_seen_frame = frame_id

                # 3. CLIP特征更新
                matched_obj.clip_feature = (0.2 * det.clip_feature +
                                           0.8 * matched_obj.clip_feature)
            else:
                # 4. 创建新实例
                new_obj = Object3D(
                    id=self._next_id(),
                    label=det.label,
                    position=det.position,
                    clip_feature=det.clip_feature,
                    score=det.score,
                    detection_count=1,
                    first_seen_frame=frame_id
                )
                self.objects.append(new_obj)

        # 5. 更新空间关系
        self._update_relations()

        # 6. 区域聚类
        self._update_regions()

    def _associate(self, detection: Detection3D) -> Optional[Object3D]:
        """
        数据关联：匹配检测到已有实例
        """
        candidates = []
        for obj in self.objects:
            # 距离阈值
            distance = np.linalg.norm(detection.position - obj.position)
            if distance > 1.0:  # 超过1米，不可能是同一物体
                continue

            # 标签匹配
            if detection.label != obj.label:
                continue

            # CLIP特征相似度
            clip_sim = cosine_similarity(detection.clip_feature, obj.clip_feature)

            # 综合评分
            score = 0.5 * (1.0 - distance) + 0.5 * clip_sim
            candidates.append((obj, score))

        if candidates:
            best_obj, best_score = max(candidates, key=lambda x: x[1])
            if best_score > 0.6:  # 关联阈值
                return best_obj

        return None

    def _update_relations(self):
        """
        更新空间关系（8种关系）
        """
        self.relations = []
        for i, obj1 in enumerate(self.objects):
            for obj2 in self.objects[i+1:]:
                distance = np.linalg.norm(obj1.position - obj2.position)

                # near关系
                if distance < 1.5:
                    self.relations.append({
                        'subject_id': obj1.id,
                        'relation': 'near',
                        'object_id': obj2.id,
                        'distance': distance
                    })

                # left_of/right_of关系
                relative_pos = obj2.position - obj1.position
                angle = np.arctan2(relative_pos[1], relative_pos[0])
                if -np.pi/4 < angle < np.pi/4:
                    relation = 'right_of'
                elif 3*np.pi/4 < angle or angle < -3*np.pi/4:
                    relation = 'left_of'
                elif angle > 0:
                    relation = 'in_front_of'
                else:
                    relation = 'behind'

                self.relations.append({
                    'subject_id': obj1.id,
                    'relation': relation,
                    'object_id': obj2.id
                })

    def _update_regions(self):
        """
        区域聚类（DBSCAN-like）
        """
        # 简化版：按3米半径分组
        self.regions = []
        unassigned = set(obj.id for obj in self.objects)
        region_id = 0

        while unassigned:
            seed_id = unassigned.pop()
            seed_obj = self.get_object(seed_id)
            region_objects = [seed_id]

            # 扩展区域
            for obj in self.objects:
                if obj.id in unassigned:
                    distance = np.linalg.norm(obj.position - seed_obj.position)
                    if distance < 3.0:
                        region_objects.append(obj.id)
                        unassigned.remove(obj.id)

            # 创建区域
            self.regions.append({
                'region_id': region_id,
                'object_ids': region_objects,
                'name': f"area_with_{seed_obj.label}"
            })
            region_id += 1
```

## 5. LOVON动作原语

### 6种动作原语
```python
class ActionExecutor:
    """
    LOVON (2024) 四足机器人VLN动作原语
    """
    def execute_navigate(self, target_position, target_label):
        """
        NAVIGATE: 导航到目标位置
        """
        # 计算朝向（自动对准目标）
        direction = target_position - robot_pose.position
        yaw = np.arctan2(direction[1], direction[0])

        goal = PoseStamped()
        goal.pose.position.x = target_position[0]
        goal.pose.position.y = target_position[1]
        goal.pose.orientation = yaw_to_quaternion(yaw)

        self.nav_goal_pub.publish(goal)

    def execute_approach(self, target_position, stop_distance=0.5):
        """
        APPROACH: 接近目标（最后0.5m，减速）
        """
        direction = normalize(target_position - robot_pose.position)
        approach_position = target_position - direction * stop_distance

        goal = PoseStamped()
        goal.pose.position.x = approach_position[0]
        goal.pose.position.y = approach_position[1]
        # 设置减速标志
        goal.header.frame_id = "approach_mode"

        self.nav_goal_pub.publish(goal)

    def execute_look_around(self, duration=12.0):
        """
        LOOK_AROUND: 原地360度扫描
        """
        twist = Twist()
        twist.angular.z = 0.5  # rad/s

        # 发布旋转命令（12秒 ≈ 360度）
        rate = rospy.Rate(10)  # 10Hz
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # 停止
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def execute_verify(self, target_position):
        """
        VERIFY: 转向目标，视觉确认
        """
        direction = target_position - robot_pose.position
        yaw = np.arctan2(direction[1], direction[0])

        # 只转向，不移动
        goal = PoseStamped()
        goal.pose.position = robot_pose.position
        goal.pose.orientation = yaw_to_quaternion(yaw)

        self.nav_goal_pub.publish(goal)

        # 等待视觉确认
        time.sleep(1.0)
        return self.vision_verify(target_position)

    def execute_backtrack(self):
        """
        BACKTRACK: 回退到上一位置
        """
        prev_position = self.topological_memory.get_previous_position()
        self.execute_navigate(prev_position, "previous_location")

    def execute_explore(self, frontier_scorer):
        """
        EXPLORE: 去未知区域探索
        """
        frontiers = self.extract_frontiers()
        best_frontier = frontier_scorer.select_best(frontiers)
        self.execute_navigate(best_frontier.position, "frontier")
```

## 6. 拓扑记忆

### 核心功能
```python
class TopologicalMemory:
    """
    VLMnav (2024) + L3MVN (ICRA 2024) 拓扑记忆
    """
    def __init__(self):
        self.nodes = []  # 拓扑节点
        self.edges = []  # 导航边
        self.node_spacing = 2.0  # 每2米创建节点

    def update_position(self, position, visible_objects, clip_feature):
        """
        更新位置（可能创建新节点）
        """
        # 检查是否需要创建新节点
        if not self.nodes or self._distance_to_nearest(position) > self.node_spacing:
            node = TopologicalNode(
                id=len(self.nodes),
                position=position,
                clip_feature=clip_feature,
                visible_labels=[obj['label'] for obj in visible_objects],
                timestamp=time.time()
            )
            self.nodes.append(node)

            # 连接到最近节点
            if len(self.nodes) > 1:
                nearest = self._find_nearest_node(position)
                self.edges.append((nearest.id, node.id))

    def query_by_text(self, text_query: str) -> Optional[TopologicalNode]:
        """
        文本查询节点（CLIP匹配）
        "回到有红色沙发的房间"
        """
        query_feature = clip_encoder.encode_text(text_query)

        best_node = None
        best_score = -1.0
        for node in self.nodes:
            score = cosine_similarity(query_feature, node.clip_feature)
            if score > best_score:
                best_score = score
                best_node = node

        if best_score > 0.6:
            return best_node
        return None

    def compute_novelty(self, direction: str) -> float:
        """
        计算方向的新颖度（8扇区统计）
        """
        sector_counts = self._compute_sector_visits()
        sector_idx = self._direction_to_sector(direction)

        max_visits = max(sector_counts)
        if max_visits == 0:
            return 1.0

        novelty = 1.0 - (sector_counts[sector_idx] / max_visits)
        return novelty
```

## 总结

### 关键创新点
1. **Fast-Slow双进程**: 70%场景用Fast Path，延迟降低99.5%
2. **ESCA过滤**: tokens减少90%，推理更精准
3. **MTU3D评分**: Grounding Potential提升探索效率14-23%
4. **多源融合**: 4个信号源综合决策，鲁棒性强

### 性能目标
- Fast Path响应: <200ms
- 端到端成功率: >75%
- API费用: 降低90%
- 检测帧率: >10 FPS

### 论文级实现要点
- 完整实现论文算法，不简化
- 添加详细注释和文档
- 性能优化（TensorRT、批处理）
- 完整测试覆盖（102个测试）
- 多后端容错和降级策略
