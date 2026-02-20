"""
任务分解器 — 将复杂自然语言指令拆解为子目标序列。

参考论文:
  - SayCan (Google, 2022):      LLM 分解 → 可行性评分 → 执行
  - Inner Monologue (2022):     Chain-of-thought task decomposition
  - SG-Nav (NeurIPS 2024):      层次场景图 + 子目标规划

核心思想:
  "去厨房拿红色杯子" →
    SubGoal(action=NAVIGATE, target="kitchen area")
    SubGoal(action=FIND, target="red cup")
    SubGoal(action=APPROACH, target="red cup")
    SubGoal(action=VERIFY, target="red cup")

  每个子目标有自己的成功/失败条件和回退策略。
"""

import json
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class SubGoalAction(Enum):
    """子目标动作类型 (参考 LOVON + SayCan)。"""
    NAVIGATE = "navigate"           # 导航到指定区域/位置
    FIND = "find"                   # 在当前视野中搜索目标
    APPROACH = "approach"           # 接近已发现的目标
    VERIFY = "verify"              # 近距离验证目标身份
    LOOK_AROUND = "look_around"    # 原地旋转扫描 (LOVON)
    EXPLORE = "explore"            # 探索未知区域
    BACKTRACK = "backtrack"        # 回退到上一个位置 (LOVON)
    WAIT = "wait"                  # 等待条件满足
    FOLLOW = "follow"              # 持续跟随动态目标 (人/物体)
    STOP = "stop"                  # 停止当前任务
    PICK = "pick"                  # 抓取/取物 (Manipulation)
    PLACE = "place"                # 放置物体 (Manipulation)
    STATUS = "status"              # 查询系统/机器人状态


class SubGoalStatus(Enum):
    """子目标状态。"""
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class SubGoal:
    """单个子目标。"""
    step_id: int
    action: SubGoalAction
    target: str                     # 目标描述 ("kitchen", "red cup", etc.)
    parameters: Dict = field(default_factory=dict)  # 附加参数
    status: SubGoalStatus = SubGoalStatus.PENDING
    result: Optional[Dict] = None   # 执行结果
    retry_count: int = 0
    max_retries: int = 2

    def to_dict(self) -> Dict:
        return {
            "step_id": self.step_id,
            "action": self.action.value,
            "target": self.target,
            "status": self.status.value,
            "retry_count": self.retry_count,
        }


@dataclass
class TaskPlan:
    """完整的任务计划。"""
    instruction: str
    subgoals: List[SubGoal] = field(default_factory=list)
    current_step: int = 0

    @property
    def is_complete(self) -> bool:
        return all(
            sg.status in (SubGoalStatus.COMPLETED, SubGoalStatus.SKIPPED)
            for sg in self.subgoals
        )

    @property
    def is_failed(self) -> bool:
        return any(
            sg.status == SubGoalStatus.FAILED and sg.retry_count >= sg.max_retries
            for sg in self.subgoals
        )

    @property
    def active_subgoal(self) -> Optional[SubGoal]:
        for sg in self.subgoals:
            if sg.status in (SubGoalStatus.PENDING, SubGoalStatus.ACTIVE):
                return sg
        return None

    def advance(self):
        """标记当前子目标完成, 前进到下一个。"""
        active = self.active_subgoal
        if active:
            active.status = SubGoalStatus.COMPLETED
            self.current_step += 1

    def fail_current(self):
        """标记当前子目标失败。"""
        active = self.active_subgoal
        if active:
            active.retry_count += 1
            if active.retry_count >= active.max_retries:
                active.status = SubGoalStatus.FAILED
            else:
                active.status = SubGoalStatus.PENDING  # 允许重试

    def to_dict(self) -> Dict:
        return {
            "instruction": self.instruction,
            "total_steps": len(self.subgoals),
            "current_step": self.current_step,
            "is_complete": self.is_complete,
            "subgoals": [sg.to_dict() for sg in self.subgoals],
        }


class TaskDecomposer:
    """
    任务分解器: LLM 将自然语言 → 子目标序列。

    两种模式:
      1. LLM 分解: 发送指令给 LLM, 返回结构化子目标
      2. 规则分解: 简单指令的快速路径, 不调 LLM

    v2.0 新增:
      - KG 安全门: 规则分解前/后检查 KG 安全约束 (ConceptBot / SafeMind)
      - 可供性验证: PICK/PLACE 前查 KG 判断物体是否可操作
      - 开放词汇: 未知物体通过 KG 映射到已知概念获取属性
    """

    _knowledge_graph = None  # type: Optional[Any]

    @classmethod
    def set_knowledge_graph(cls, kg) -> None:
        """注入知识图谱 (全局设置, planner_node 启动时调用)。"""
        cls._knowledge_graph = kg

    # ══════════════════════════════════════════════════════════════
    #  规则匹配 (快速路径, 无需 LLM) — 工业级覆盖
    #
    #  设计原则:
    #    - 前缀列表按长度降序排列 (长优先匹配, 避免 "去" 吃掉 "去找")
    #    - 正则用 (?:...) 非捕获组, 只有一个 (.+?) 作为目标捕获
    #    - 覆盖: 命令/请求/疑问/口语/方言/礼貌/不耐烦/机器人专用
    # ══════════════════════════════════════════════════════════════

    import re as _re

    # ── 1. 导航前缀 (NAVIGATE) ──
    SIMPLE_NAV_PATTERNS_ZH = sorted([
        # 基础
        "去", "到", "走到", "前往", "导航到", "导航去",
        "走去", "过去", "走向", "前进到", "移动到",
        # 返回
        "回到", "返回到", "回去", "返回",
        # 组合
        "去找", "去看", "去看看", "去查看", "去检查",
        "去往", "赶到", "赶去", "赶往", "赶快去",
        "出发去", "出发到", "开到", "开去", "开往",
        # 引导
        "带路到", "带我到", "带我去", "领我到", "领我去",
        "引导到", "引导去", "引导我到", "引导我去",
        "陪我去", "陪我到",
        # 礼貌
        "请去", "请到", "请前往", "请导航到", "麻烦去",
        "帮我去", "帮我到", "帮忙去",
        # 命令 / 急促
        "立刻去", "马上去", "快去", "赶紧去", "立即前往",
        "速去", "快到", "马上到", "立刻到",
        # 机器人专用
        "移动至", "运动到", "行进到", "行走到",
        "巡航到", "自主前往", "自动前往",
        "规划路径到", "路径规划到",
    ], key=len, reverse=True)

    # ── 2. 查找前缀 (FIND) ──
    SIMPLE_FIND_PATTERNS_ZH = sorted([
        # 基础
        "找", "找到", "寻找", "搜索", "定位", "查找",
        # 口语变体
        "找一下", "找找", "找找看", "找一找",
        "搜一下", "搜一搜", "搜搜", "搜搜看",
        "定位到", "定位一下",
        "锁定", "锁定目标",
        # 确认/识别
        "确认", "确定", "确认一下",
        "发现", "看到", "识别", "检测", "探测",
        "辨认", "辨识", "认出",
        # 帮助
        "帮我找", "帮忙找", "帮我找到", "帮忙找到",
        "帮我搜", "帮忙搜", "帮我搜索",
        "帮我定位", "帮忙定位",
        # 礼貌
        "请找", "请寻找", "请搜索", "请定位",
        "麻烦找", "麻烦帮我找", "劳驾找",
        # 急促
        "快找", "赶紧找", "立刻找", "马上找",
        "快搜", "赶紧搜",
    ], key=len, reverse=True)

    # ── 3. 巡检前缀 (INSPECT → FIND + LOOK_AROUND) ──
    SIMPLE_INSPECT_PATTERNS_ZH = sorted([
        "检查", "检查一下", "巡检", "巡查",
        "查看", "查看一下", "查验", "核查",
        "检测", "检测一下", "排查", "排查一下",
        "检视", "审查", "查勘",
        "帮我检查", "帮忙检查", "请检查",
        "去检查", "去查看", "去巡检",
    ], key=len, reverse=True)

    # ── 4. 跟随前缀 (FOLLOW) ──
    SIMPLE_FOLLOW_PATTERNS_ZH = sorted([
        # 基础
        "跟着", "跟随", "跟踪", "追踪", "追着",
        "跟住", "盯着", "盯住", "尾随",
        "跟上", "追上",
        # 人称
        "跟这个人", "跟那个人", "跟他", "跟她", "跟他们",
        "跟上他", "跟上她", "追上他", "追上她",
        "跟着他", "跟着她", "跟着他走", "跟着她走",
        # 礼貌
        "帮我跟着", "帮忙跟着", "请跟着",
        "帮我跟踪", "请跟踪",
        # 命令
        "紧跟", "紧紧跟着", "一直跟着", "持续跟随",
        "不要跟丢", "别跟丢",
    ], key=len, reverse=True)

    # ── 5. 探索前缀 (EXPLORE) ──
    SIMPLE_EXPLORE_PATTERNS_ZH = sorted([
        "探索", "探索一下", "逛逛", "逛一逛",
        "四处看看", "到处看看", "环顾四周",
        "看看周围", "看看附近", "巡视",
        "扫描", "扫描一下", "扫描周围",
        "勘察", "侦察", "侦查",
        "帮我探索", "自由探索",
        "随便走走", "随便逛逛",
    ], key=len, reverse=True)

    # ── 6. 停止/取消前缀 ──
    SIMPLE_STOP_PATTERNS_ZH = sorted([
        "停", "停下", "停止", "停下来", "停一下",
        "别走了", "别动", "不要动", "站住",
        "取消", "取消任务", "终止", "终止任务",
        "中断", "中止", "暂停",
        "算了", "不用了", "不找了", "不去了",
        "回来", "回来吧", "不用去了",
        "紧急停止", "急停",
        "停止任务", "停掉", "关掉",
    ], key=len, reverse=True)

    # ── 7. 取物前缀 (PICK → FIND + APPROACH + PICK) ──
    SIMPLE_PICK_PATTERNS_ZH = sorted([
        "拿", "取", "拿一下", "取一下",
        "拿个", "取个", "拿一个", "取一个",
        "拿过来", "取过来", "拿来", "取来",
        "帮我拿", "帮我取", "帮忙拿", "帮忙取",
        "帮我拿一下", "帮我取一下",
        "帮我拿来", "帮我取来",
        "帮我带来", "帮忙带来",
        "给我拿", "给我取", "给我拿个", "给我取个",
        "给我拿一个", "给我取一个",
        "递给我", "递一下", "递过来",
        "请拿", "请取", "麻烦拿", "麻烦取",
        "快拿", "赶紧拿", "马上拿",
        "抓", "抓住", "抓取", "夹取", "夹住",
        "捡", "捡起", "捡起来", "捡一下",
        "把X拿来", "把X给我",
    ], key=len, reverse=True)

    # ── 8. 放置前缀 (PLACE → NAVIGATE + PLACE) ──
    SIMPLE_PLACE_PATTERNS_ZH = sorted([
        "放", "放下", "放到", "放在", "放回",
        "放一下", "放好", "放回去",
        "摆到", "摆在", "摆好", "摆放",
        "放回原处", "归位", "还回去",
        "搁到", "搁在", "搁下",
        "帮我放", "帮忙放", "帮我放到", "帮我放在",
        "请放到", "请放在",
        "放置", "放置到", "放置在",
        "丢到", "丢在", "扔到", "扔在",
        "把X放到", "把X放在", "把X放回",
    ], key=len, reverse=True)

    # ── 9. 状态查询前缀 (STATUS → 返回系统状态) ──
    SIMPLE_STATUS_PATTERNS_ZH = sorted([
        "电量", "电池", "电池电量", "电量多少", "还有多少电",
        "状态", "系统状态", "当前状态", "机器人状态",
        "模式", "当前模式", "什么模式",
        "当前任务", "任务状态", "任务进度",
        "完成了吗", "做完了吗", "好了吗",
        "还剩多少", "剩余电量", "剩余时间",
        "现在在哪", "现在位置", "当前位置", "你在哪",
        "温度", "温度多少", "当前温度",
        "是否在线", "连接状态", "网络状态",
        "速度", "当前速度",
        "报告状态", "汇报状态",
    ], key=len, reverse=True)

    # ── 10. 中文口语化查找 — 正则匹配, 提取核心目标 ──
    CONVERSATIONAL_FIND_RE_ZH = [
        # --- 查看类 ---
        _re.compile(r"(?:帮我|请|能|能不能|可以|麻烦)?(?:看看|看一下|看一看|查一下|查一查|查看|查看一下|瞧瞧|瞧一下|瞅瞅|望望|看下|查下|瞅一眼|瞧一瞧)(.+?)(?:在哪|在哪里|在哪儿|在什么地方|的位置|在什么位置|在不在)?$"),
        # --- 位置疑问 ---
        _re.compile(r"(.+?)在哪(?:里|儿)?(?:啊|呢|吗|呀|嘛|哦|嗯)?$"),
        _re.compile(r"(.+?)在什么(?:地方|位置|方向|方位)(?:啊|呢|吗)?$"),
        _re.compile(r"(.+?)(?:的位置|的方位|的方向)(?:在哪|是什么)?(?:啊|呢|吗)?$"),
        # --- 帮助类 ---
        _re.compile(r"(?:帮我|请|能不能|可以|麻烦|劳驾|烦请)?(?:找一下|找一找|帮忙找|帮忙找一下|帮忙搜一下|帮找)(.+?)(?:吧|好吗|可以吗|行吗)?$"),
        # --- 引导类 ---
        _re.compile(r"(?:带我去|带我到|领我去|领我到|带路去|带路到|引导我去|引导我到|陪我去|引我去|送我去|送我到)(.+?)(?:吧|好吗)?$"),
        # --- 意愿类 ---
        _re.compile(r"(?:我想|我要|我想去|我要去|我得去|我得找|我需要找|我需要去|我想要|我希望|我打算去|我打算找|我准备去|我准备找|咱们去|我们去)(?:找|看|去|找到|看看|看一下|去看)?(.+?)$"),
        # --- 存在疑问 ---
        _re.compile(r"(?:哪里有|哪儿有|什么地方有|哪边有|哪有|有没有|有没有看到|附近有没有|周围有没有|能看到|看得到)(.+?)(?:吗|呢|啊)?$"),
        # --- 路径疑问 ---
        _re.compile(r"(.+?)(?:怎么走|怎么去|怎么找|怎么到|如何去|如何找|如何到达|怎么过去|咋走|咋去|咋找)(?:啊|呢|吗)?$"),
        # --- 指示类 ---
        _re.compile(r"(?:告诉我|告诉一下|指给我看|指一下|给我看看|给我指|给我指一下|帮我指|报告|汇报|说一下|说说)(.+?)(?:在哪|的位置|的方向)?(?:吧|好吗)?$"),
        # --- 位置确认 ---
        _re.compile(r"(.+?)(?:是在|在|是|到底在)(?:哪个位置|哪个地方|什么地方|什么位置|哪边|哪个方向|哪个方位)(?:啊|呢|吗)?$"),
        # --- 存在确认 ---
        _re.compile(r"(?:这里|这边|这儿|附近|周围|旁边|楼里|室内|室外|前面|后面)?有(.+?)(?:吗|没|没有|嘛)$"),
        # --- 能力疑问 ---
        _re.compile(r"(?:能|能不能|可以|能否|可否|行不行|能不|可不可以)(?:找到|找|看到|看|定位|发现|搜到|搜索到|检测到|识别)(.+?)(?:吗|嘛|呢|啊)?$"),
        # --- 距离/方向 ---
        _re.compile(r"(?:最近的|离我最近的|距离最近的|附近的|身边的|旁边的|面前的|眼前的)(.+?)(?:在哪|的位置)?$"),
        # --- 数量确认 ---
        _re.compile(r"(?:有几个|有多少|有多少个|一共有多少)(.+?)(?:啊|呢|吗)?$"),
        # --- 对话式 ---
        _re.compile(r"(?:你知道|你看到|你看见|你发现|你检测到|你有没有看到|你有没有发现)(.+?)(?:吗|了吗|没|没有|在哪)?$"),
        # --- 催促式 ---
        _re.compile(r"(?:快|赶紧|赶快|马上|立刻|立马|速速|抓紧)(?:找|去找|帮我找|搜|去搜|定位|去看|去查)(.+?)$"),
        # --- "给我找X" (FIND, 不是 PICK) ---
        _re.compile(r"(?:给我找|给我找个|给我找一个|给我搜|帮我找个|帮我找一个)(.+?)$"),
        # --- 否定排除式 ---
        _re.compile(r"(?:不是|不要|别找)(?:.+?)(?:是|要|找|去找)(.+?)$"),
        # --- 方言 / 口头禅 ---
        _re.compile(r"(?:整个|搞个|弄个|来个|搞一个|弄一个)(.+?)(?:来|过来|出来)?$"),
        # --- 场景感知 ---
        _re.compile(r"(?:这里|这边|附近|周围|前面|后面)(?:都)?有(?:些|哪些|什么)(.+?)$"),
    ]

    # ── 11. 英文口语化查找 — 正则匹配 ──
    CONVERSATIONAL_FIND_RE_EN = [
        # --- Command / Request ---
        _re.compile(r"(?:can you |could you |would you |please |hey |ok |okay )?(?:show me|take me to|bring me to|lead me to|walk me to|escort me to) (?:the |a |an )?(.+?)(?:\?|\.)?$", _re.IGNORECASE),
        _re.compile(r"(?:can you |could you |would you |please )?(?:point (?:me )?(?:to|at|towards)|direct me to|guide me to|navigate (?:me )?to) (?:the |a |an )?(.+?)(?:\?|\.)?$", _re.IGNORECASE),
        # --- Question ---
        _re.compile(r"where (?:is|are|can i find|do i find|would i find|might i find|shall i find) (?:the |a |an )?(.+?)(?:\?)?$", _re.IGNORECASE),
        _re.compile(r"(?:where\'s|where are) (?:the |a |an )?(.+?)(?:\?)?$", _re.IGNORECASE),
        _re.compile(r"(?:do you know where|do you see|can you see|have you seen|did you see|did you spot) (?:the |a |an )?(.+?)(?:\?| is)?$", _re.IGNORECASE),
        # --- Desire / Need ---
        _re.compile(r"(?:i need|i want|i\'d like|i would like|i\'m looking for|i am looking for|i gotta find|i must find|i have to find) (?:to find |to see |to go to |to get to |to reach |to locate )?(?:the |a |an )?(.+?)(?:\.|!)?$", _re.IGNORECASE),
        # --- Existence ---
        _re.compile(r"(?:is there|are there|do we have|does this (?:place|building|floor) have) (?:a |an |any |some )?(.+?)(?:\s+)?(?:here|nearby|around|close by|in (?:this|the) (?:building|room|area|floor))?(?:\?)?$", _re.IGNORECASE),
        # --- How to ---
        _re.compile(r"(?:how do i (?:get to|find|reach|locate)|how can i (?:find|get to|reach)|how to (?:find|get to|reach|locate)|what\'s the way to) (?:the |a |an )?(.+?)(?:\?)?$", _re.IGNORECASE),
        # --- Help ---
        _re.compile(r"(?:help me (?:find|locate|get to|reach|look for)|help (?:find|locate)) (?:the |a |an )?(.+?)(?:\.|!)?$", _re.IGNORECASE),
        # --- Informal / Collaborative ---
        _re.compile(r"(?:let me see|let\'s find|let\'s go (?:to|find)|let\'s look for|let\'s check(?: out)?|let\'s head to|lemme see|lemme find) (?:the |a |an )?(.+?)$", _re.IGNORECASE),
        # --- Imperative ---
        _re.compile(r"(?:go (?:find|get|fetch|grab)|fetch me|get me|bring me|grab|gimme) (?:the |a |an )?(.+?)$", _re.IGNORECASE),
        # --- Closest / Nearest ---
        _re.compile(r"(?:(?:find|locate|show me|where is|where\'s) )?(?:the )?(?:nearest|closest|closest available|nearest available) (.+?)(?:\?)?$", _re.IGNORECASE),
        # --- Confirmation ---
        _re.compile(r"(?:is|are) (?:the |that |this )?(.+?) (?:around here|over there|near(?:by)?|still there|in this (?:room|area|building))(?:\?)?$", _re.IGNORECASE),
        # --- Robot-specific ---
        _re.compile(r"(?:scan for|search for|detect|identify|locate|look for) (?:the |a |an |any )?(.+?)$", _re.IGNORECASE),
        _re.compile(r"(?:report|check) (?:the )?(?:location|position|status) (?:of )?(?:the |a |an )?(.+?)$", _re.IGNORECASE),
        # --- Informal slang ---
        _re.compile(r"(?:swing by|drop by|pop over to|head over to) (?:the |a |an )?(.+?)$", _re.IGNORECASE),
        # --- Scene awareness ---
        _re.compile(r"what(?:\'s| is| do you see) (?:around here|nearby|over there|in (?:this|the) (?:room|area))(?:\?)?$", _re.IGNORECASE),
        _re.compile(r"(?:anything|something) (?:interesting |useful )?(?:nearby|around|here)(?:\?)?$", _re.IGNORECASE),
        _re.compile(r"what (?:can you|do you) (?:see|detect|spot)(?:\?)?$", _re.IGNORECASE),
    ]

    # ── 12. 英文导航前缀 ──
    SIMPLE_NAV_PATTERNS_EN = [
        "go to ", "navigate to ", "move to ", "head to ", "walk to ",
        "proceed to ", "drive to ", "travel to ", "advance to ",
        "return to ", "go back to ", "head back to ",
        "rush to ", "hurry to ", "quickly go to ",
        "head over to ", "swing by ", "drop by ", "pop over to ",
    ]

    # ── 13. 英文跟随前缀 ──
    SIMPLE_FOLLOW_PATTERNS_EN = [
        "follow ", "track ", "chase ", "tail ",
        "keep following ", "keep tracking ",
        "stay with ", "stick with ",
        "pursue ", "shadow ",
    ]

    # ── 14. 英文取物前缀 ──
    SIMPLE_PICK_PATTERNS_EN = [
        "pick up ", "grab ", "fetch ", "fetch me ",
        "bring me ", "get me ", "hand me ", "gimme ",
        "go grab ", "go get ", "go fetch ",
        "please grab ", "please fetch ",
    ]

    # ── 15. 英文放置前缀 ──
    SIMPLE_PLACE_PATTERNS_EN = [
        "put ", "place ", "drop ", "set down ",
        "put down ", "leave ", "return ",
        "put it on ", "place it on ", "set it on ",
    ]

    # ── 16. 英文状态查询 ──
    SIMPLE_STATUS_PATTERNS_EN = [
        "battery level", "battery status", "how much battery",
        "current status", "system status", "robot status",
        "where are you", "your location", "current position",
        "what mode", "current mode",
        "task status", "task progress", "are you done",
        "remaining battery", "remaining time",
        "temperature", "current speed",
        "connection status", "network status",
        "report status", "status report",
    ]

    # ── 17. 英文巡检前缀 ──
    SIMPLE_INSPECT_PATTERNS_EN = [
        "inspect ", "examine ", "audit ",
    ]

    def decompose_with_rules(self, instruction: str) -> Optional[TaskPlan]:
        """
        规则分解 (快速路径)。

        简单指令如 "去门那里" 不需要 LLM, 直接生成:
          NAVIGATE → target → APPROACH → VERIFY

        跟随指令如 "跟着那个人" 生成:
          FIND → FOLLOW (持续)

        Returns:
            TaskPlan or None (if too complex for rules)
        """
        inst = instruction.strip()

        # ── 复杂度守卫: 含条件/多步/逻辑关系/时间约束 → 直接走 LLM ──
        _COMPLEXITY_MARKERS_ZH = (
            "如果", "否则", "然后再", "接着再", "之后再",
            "并且", "而且", "同时", "以及",
            "先去", "先找", "先到",
            "每个", "每一个", "所有", "全部", "依次",
            "巡逻", "巡视所有", "逐一",
            "完成后", "完成之后", "完成以后", "做完再",
            "然后", "接着", "再去", "再找",
            "每隔", "定期", "循环", "反复",
        )
        _COMPLEXITY_MARKERS_EN = (
            " if ", " then ", " else ", " otherwise ", " and then ",
            " after that ", " followed by ", " every ", " each ",
            " all ", " patrol ", " one by one ",
            " after ", " before ", " once done ",
            " repeat ", " periodically ", " continuously ",
        )
        if any(m in inst for m in _COMPLEXITY_MARKERS_ZH):
            return None
        if any(m in f" {inst.lower()} " for m in _COMPLEXITY_MARKERS_EN):
            return None
        if inst.count("，") >= 2 or inst.count(",") >= 2:
            return None

        inst_lower = inst.lower()

        # ── 0. 停止/取消 (最高优先级) ──
        is_stop = any(inst.startswith(p) for p in self.SIMPLE_STOP_PATTERNS_ZH)
        if inst_lower.rstrip("!. ") in ("stop", "halt", "cancel", "abort", "quit", "enough",
                                          "nevermind", "never mind", "forget it"):
            is_stop = True
        if is_stop:
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.STOP, target="current_task")],
            )

        # ── 1. 状态查询 (不涉及运动, 立即返回) ──
        is_status = any(inst.startswith(p) for p in self.SIMPLE_STATUS_PATTERNS_ZH)
        if any(inst_lower.startswith(p) or inst_lower == p for p in self.SIMPLE_STATUS_PATTERNS_EN):
            is_status = True
        if is_status:
            query = inst
            for p in self.SIMPLE_STATUS_PATTERNS_ZH:
                if query.startswith(p):
                    query = p
                    break
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.STATUS, target=query)],
            )

        # ── 2. 探索 ──
        is_explore = any(inst.startswith(p) for p in self.SIMPLE_EXPLORE_PATTERNS_ZH)
        if inst_lower.startswith(("explore", "look around", "scan the", "survey", "wander")):
            is_explore = True
        if is_explore:
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.EXPLORE,
                            target="environment", parameters={"strategy": "frontier"}),
                ],
            )

        # ── 3. 放置 (PLACE: 含"把X放到Y") ──
        is_place = any(inst.startswith(p) for p in self.SIMPLE_PLACE_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_PLACE_PATTERNS_EN):
            is_place = True
        if is_place:
            target = inst
            for p in self.SIMPLE_PLACE_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p):].strip()
                    break
            for p in self.SIMPLE_PLACE_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p):].strip()
                    break
            if not target or target == inst:
                target = "current_location"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=target),
                    SubGoal(step_id=1, action=SubGoalAction.PLACE, target=target),
                ],
            )

        # ── 4. 取物 (PICK: FIND + APPROACH + PICK) ──
        is_pick = any(inst.startswith(p) for p in self.SIMPLE_PICK_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_PICK_PATTERNS_EN):
            is_pick = True
        if is_pick:
            target = inst
            for p in self.SIMPLE_PICK_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p):].strip()
                    break
            for p in self.SIMPLE_PICK_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p):].strip()
                    break
            if not target or target == inst:
                target = "object"

            # KG 安全门 (ConceptBot / SafeMind): PICK 前检查物体可操作性
            kg_params = {}
            if self._knowledge_graph is not None:
                manip_info = self._knowledge_graph.get_manipulation_info(target, "pick")
                if not manip_info.get("feasible", True):
                    reason = manip_info.get("reason", "unknown")
                    notes = manip_info.get("notes", [])
                    logger.warning(
                        "KG safety gate BLOCKED pick '%s': %s — %s",
                        target, reason, notes,
                    )
                    return TaskPlan(
                        instruction=instruction,
                        subgoals=[
                            SubGoal(
                                step_id=0,
                                action=SubGoalAction.STATUS,
                                target=target,
                                parameters={
                                    "kg_blocked": True,
                                    "reason": reason,
                                    "notes": notes,
                                    "safety": manip_info.get("safety", "unknown"),
                                },
                            ),
                        ],
                    )
                kg_params = {
                    "kg_safety": manip_info.get("safety", "safe"),
                    "kg_weight": manip_info.get("weight_range", []),
                    "kg_size": manip_info.get("size_class", "unknown"),
                }

            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(step_id=1, action=SubGoalAction.APPROACH, target=target,
                            parameters={"approach_distance": 0.3, **kg_params}),
                    SubGoal(step_id=2, action=SubGoalAction.PICK, target=target,
                            parameters=kg_params),
                ],
            )

        # ── 5. 巡检 (INSPECT: FIND + LOOK_AROUND + APPROACH + VERIFY) ──
        #    优先级 > FIND, 避免 "去检查灭火器" 被 NAV 吃掉
        is_inspect = any(inst.startswith(p) for p in self.SIMPLE_INSPECT_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_INSPECT_PATTERNS_EN):
            is_inspect = True
        if inst_lower.startswith("check ") and not inst_lower.startswith(("check if ", "check whether ")):
            is_inspect = True
        if is_inspect:
            target = inst
            for p in self.SIMPLE_INSPECT_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p):].strip()
                    break
            for p in list(self.SIMPLE_INSPECT_PATTERNS_EN) + ["check "]:
                if inst_lower.startswith(p):
                    target = inst[len(p):].strip()
                    break
            if not target or target == inst:
                target = "environment"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(step_id=1, action=SubGoalAction.LOOK_AROUND, target=target,
                            parameters={"reason": "inspect target"}),
                    SubGoal(step_id=2, action=SubGoalAction.APPROACH, target=target),
                    SubGoal(step_id=3, action=SubGoalAction.VERIFY, target=target),
                ],
            )

        # ── 6. 跟随 ──
        is_follow = any(inst.startswith(p) or p in inst for p in self.SIMPLE_FOLLOW_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_FOLLOW_PATTERNS_EN):
            is_follow = True
        if "follow" in inst_lower and ("person" in inst_lower or "human" in inst_lower or "him" in inst_lower or "her" in inst_lower):
            is_follow = True

        if is_follow:
            target = inst
            for p in self.SIMPLE_FOLLOW_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p):].strip()
                    break
            for p in self.SIMPLE_FOLLOW_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p):].strip()
                    break
            if not target or target == inst:
                target = "person"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(
                        step_id=1,
                        action=SubGoalAction.FOLLOW,
                        target=target,
                        parameters={"follow_distance": 1.5, "timeout": 300},
                        max_retries=5,
                    ),
                ],
            )

        # ── 7. 简单导航 / 查找 ──
        is_simple_nav = any(inst.startswith(p) for p in self.SIMPLE_NAV_PATTERNS_ZH)
        is_simple_find = any(inst.startswith(p) for p in self.SIMPLE_FIND_PATTERNS_ZH)

        if any(inst_lower.startswith(p) for p in self.SIMPLE_NAV_PATTERNS_EN):
            is_simple_nav = True
        if inst_lower.startswith(("find ", "search for ", "locate ", "look for ",
                                   "where is ", "where are ", "show me ",
                                   "seek ", "spot ", "identify ")):
            is_simple_find = True

        # 口语化匹配: "看一下灭火器在哪", "灭火器在哪", "where is the door" 等
        conversational_target = None
        if not is_simple_nav and not is_simple_find:
            for pat in self.CONVERSATIONAL_FIND_RE_ZH:
                m = pat.search(inst)
                if m:
                    conversational_target = m.group(1).strip()
                    if conversational_target:
                        is_simple_find = True
                        break
            if not is_simple_find:
                for pat in self.CONVERSATIONAL_FIND_RE_EN:
                    m = pat.search(inst)
                    if m:
                        conversational_target = m.group(1).strip()
                        if conversational_target:
                            is_simple_find = True
                            break

        if not is_simple_nav and not is_simple_find:
            return None  # 需要 LLM 分解

        # 提取目标 (去掉动词前缀)
        if conversational_target:
            target = conversational_target
        else:
            target = inst
            for p in self.SIMPLE_NAV_PATTERNS_ZH + self.SIMPLE_FIND_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p):].strip()
                    break
            for p in self.SIMPLE_NAV_PATTERNS_EN + [
                "find ", "search for ", "locate ", "look for ",
                "where is ", "where are ", "show me ",
                "seek ", "spot ", "identify ",
            ]:
                if inst_lower.startswith(p):
                    target = inst[len(p):].strip()
                    break

        subgoals = []
        step = 0

        # KG 知识增强: 查询目标的安全约束和典型位置
        kg_approach_params: Dict = {"approach_distance": 0.5}
        kg_find_params: Dict = {}
        if self._knowledge_graph is not None:
            # 安全距离约束 (SafeMind)
            constraint = self._knowledge_graph.check_safety(target, "approach")
            if constraint and constraint.max_approach_distance > 0:
                kg_approach_params["approach_distance"] = max(
                    kg_approach_params["approach_distance"],
                    constraint.max_approach_distance,
                )
                kg_approach_params["kg_safety_note"] = constraint.message_en

            # 典型位置提示 (ConceptBot: 引导探索方向)
            typical_locs = self._knowledge_graph.get_typical_locations(target)
            if typical_locs:
                kg_find_params["typical_locations"] = typical_locs[:5]

            # 安全等级标注
            safety = self._knowledge_graph.get_safety_level(target)
            kg_approach_params["kg_safety"] = safety.value

        if is_simple_find:
            subgoals.append(SubGoal(
                step_id=step, action=SubGoalAction.FIND, target=target,
                parameters=kg_find_params,
            ))
            step += 1
            subgoals.append(SubGoal(
                step_id=step, action=SubGoalAction.LOOK_AROUND, target=target,
                parameters={"reason": "scan for target"},
            ))
            step += 1

        subgoals.append(SubGoal(
            step_id=step, action=SubGoalAction.NAVIGATE, target=target,
        ))
        step += 1
        subgoals.append(SubGoal(
            step_id=step, action=SubGoalAction.APPROACH, target=target,
            parameters=kg_approach_params,
        ))
        step += 1
        subgoals.append(SubGoal(
            step_id=step, action=SubGoalAction.VERIFY, target=target,
        ))

        return TaskPlan(instruction=instruction, subgoals=subgoals)

    def build_decomposition_prompt(
        self,
        instruction: str,
        scene_summary: str = "",
        language: str = "zh",
    ) -> List[Dict[str, str]]:
        """
        构建 LLM 分解 prompt (SayCan / Inner Monologue 风格)。

        Args:
            instruction: 用户指令
            scene_summary: 场景摘要
            language: "zh" / "en"

        Returns:
            messages list for LLM
        """
        available_actions = ", ".join(a.value for a in SubGoalAction)

        if language == "zh":
            system = f"""你是一个机器人任务规划器。将用户的自然语言指令分解为一系列可执行的子目标。

可用动作类型: {available_actions}

规则:
1. 每个子目标必须是原子操作 (单一动作)
2. navigate: 导航到一个区域或位置
3. find: 在场景图中搜索匹配物体
4. approach: 接近已发现的目标 (最后 0.5m)
5. verify: 近距离确认目标身份
6. look_around: 原地 360° 扫描
7. explore: 去未探索区域搜索
8. backtrack: 回到上一位置
9. wait: 等待 (用于动态场景)

输出格式 (严格 JSON):
{{
  "subgoals": [
    {{"action": "navigate", "target": "...", "parameters": {{}}}},
    ...
  ]
}}"""
            user_content = f"## 指令\n{instruction}"
            if scene_summary:
                user_content += f"\n\n## 当前场景\n{scene_summary}"
        else:
            system = f"""You are a robot task planner. Decompose the user's instruction into a sequence of executable subgoals.

Available actions: {available_actions}

Rules:
1. Each subgoal must be atomic (single action)
2. navigate: go to a region or position
3. find: search for matching object in scene graph
4. approach: move to within 0.5m of target
5. verify: close-range identity confirmation
6. look_around: 360° scan in place
7. explore: go to unexplored area
8. backtrack: return to previous position
9. wait: wait for condition

Output format (strict JSON):
{{
  "subgoals": [
    {{"action": "navigate", "target": "...", "parameters": {{}}}},
    ...
  ]
}}"""
            user_content = f"## Instruction\n{instruction}"
            if scene_summary:
                user_content += f"\n\n## Current Scene\n{scene_summary}"

        return [
            {"role": "system", "content": system},
            {"role": "user", "content": user_content},
        ]

    def parse_decomposition_response(
        self, instruction: str, response_text: str
    ) -> TaskPlan:
        """
        解析 LLM 返回的子目标列表。

        Args:
            instruction: 原始指令
            response_text: LLM 响应文本

        Returns:
            TaskPlan
        """
        import re

        # 提取 JSON
        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", response_text)
        if match:
            raw = match.group(1).strip()
        else:
            start = response_text.find("{")
            end = response_text.rfind("}")
            if start == -1 or end == -1:
                logger.error("No JSON in decomposition response: %s", response_text[:200])
                # Fallback: 单步导航
                return TaskPlan(
                    instruction=instruction,
                    subgoals=[
                        SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                        SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                    ],
                )
            raw = response_text[start:end + 1]

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.error("JSON parse error in decomposition: %s", e)
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                    SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                ],
            )

        subgoals = []
        for i, sg_data in enumerate(data.get("subgoals", [])):
            action_str = sg_data.get("action", "navigate")
            try:
                action = SubGoalAction(action_str)
            except ValueError:
                action = SubGoalAction.NAVIGATE

            subgoals.append(SubGoal(
                step_id=i,
                action=action,
                target=sg_data.get("target", instruction),
                parameters=sg_data.get("parameters", {}),
            ))

        if not subgoals:
            subgoals = [
                SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
            ]

        return TaskPlan(instruction=instruction, subgoals=subgoals)
