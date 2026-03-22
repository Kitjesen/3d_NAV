# semantic_planner — VLN 语义规划 ROS2 包
# 模块归属详见同目录 MODULES.md

# --- 根节点 ---
# planner_node.py       ROS2 主节点
# planner_state.py      PlannerState 枚举

# --- Agent (agent_node / skill_registry / mcp_server) ---
# agent_node.py         多轮对话 LLM Agent
# skill_registry.py     @skill 装饰器 + 注册中心
# mcp_server.py         MCP 协议服务端

# --- Goal 目标解析 (goal_resolver / fast_path / slow_path / adacot) ---
# goal_resolver.py      Fast-Slow 协调器，AdaNav 熵触发
# fast_path.py          System 1 场景图匹配 <200ms
# slow_path.py          System 2 LLM 推理 ~2s
# adacot.py             AdaCoT 自适应推理路由

# --- Execution 执行 (action_executor / bbox_navigator / vlm_bbox_query / person_tracker / exploration_strategy) ---
# action_executor.py    动作原语 + LERa 3 步恢复
# bbox_navigator.py     视觉伺服导航
# vlm_bbox_query.py     VLM 包围框查询
# person_tracker.py     行人跟踪 + 跟随
# exploration_strategy.py  Frontier 覆盖调度

# --- Task 任务分解 (task_decomposer / task_rules) ---
# task_decomposer.py    SayCan 风格任务分解
# task_rules.py         场所-技能规则库

# --- Memory 记忆 (episodic_memory / topological_memory / tagged_locations / semantic_prior) ---
# episodic_memory.py    ReMEmbR 情节记忆 500 条 FIFO
# topological_memory.py 拓扑地图 + FSR-VLN 边 + VLingMem 摘要
# tagged_locations.py   命名地点持久化
# semantic_prior.py     房间语义先验 + predict_room_type

# --- Frontier 探索推理 (frontier_scorer / frontier_types / sgnav_reasoner) ---
# frontier_scorer.py    MTU3D + L3MVN + TSP 排序
# frontier_types.py     FrontierNode / FrontierCluster 数据类型
# sgnav_reasoner.py     SGNav 多视角场景图推理

# --- LLM 客户端 (llm_client / prompt_templates / chinese_tokenizer) ---
# llm_client.py         多后端 LLM，自动 fallback，流式
# prompt_templates.py   H-CoT 4 步 + frontier/explored prompt
# chinese_tokenizer.py  jieba 分词 + 停用词

# --- Mixin (planner_init / nav2_mixin / subgoal_mixin / bbox_mixin / operational_mixin / resolve_mixin / callbacks_mixin / state_mixin) ---
# planner_init.py       初始化：ROS2 声明 + 子模块实例化
# nav2_mixin.py         Nav2 ActionClient 封装
# subgoal_mixin.py      子目标队列推进
# bbox_mixin.py         BBoxNavigator 生命周期
# operational_mixin.py  健康检查 + metrics
# resolve_mixin.py      goal_resolver 调用封装
# callbacks_mixin.py    ROS2 回调实现
# state_mixin.py        PlannerState FSM 转换

# --- Viz (rerun_viewer) ---
# rerun_viewer.py       Rerun SDK 实时可视化

# --- Utils (implicit_fsm_policy / voi_scheduler / room_object_kg) ---
# implicit_fsm_policy.py   LOVON 隐式 FSM 策略
# voi_scheduler.py         VOI 感知查询调度
# room_object_kg.py        房间-物体知识图谱先验
