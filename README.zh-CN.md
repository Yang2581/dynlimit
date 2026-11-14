# DynLimit v0.1.1

[English](README.md)

**DynLimit**（*Dynamics at the Limit*）是面向车辆动力学极限与极限工况的仿真与评估平台：

- **Python**：仿真时钟、规划/控制、日志、HTTP API
- **Web UI**：Live / Replay / Edit / Assets
- **CarSim（可选）**：自备许可与数据集时使用高保真车辆模型

本目录是**稳定的终端用户发布快照**。默认用 **Mock** plant 即可运行，**不需要** CarSim。

### v0.1.1 更新

- 车道局部规划控制器：`lane_pp`、`lattice_pid_ml`（以及既有 `lattice_pid` / `strategy01`）
- 演示资源：`4highway` 赛道 + `4highway_demo` 场景
- 通过 `dynlimit.toml` 的 `[paths].carsim_db` 配置 CarSim 数据集路径（仍可用 CLI/环境变量覆盖）

## 环境要求

- Python ≥ 3.10
- Node.js + npm（Web 界面）
- 可选：Mechanical Simulation CarSim 许可、本机 `carsim_db`、可导入的 `pycarsimlib`

## 快速开始（Mock）

在本目录（`opensource/v0.1.1/`）下：

```powershell
# 1) Python 环境
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install -U pip
python -m pip install -e .

# 2) 前端依赖（首次）
cd frontend
npm install
cd ..
```

**终端 1 — API**

```powershell
.\.venv\Scripts\activate
dynlimit-api
# 或：python -m backend.app.main
```

- API：http://127.0.0.1:8000  
- OpenAPI：http://127.0.0.1:8000/docs  

**终端 2 — UI**

```powershell
cd frontend
npm run dev
```

打开 **http://127.0.0.1:5173**（请用 Vite 端口，不要用 8000）。

在 **Live** 中：选择演示场景（如 `web_demo_001` 或 `4highway_demo`）→ Create → Start，plant 选 Mock。

## 界面页签

| 页签 | 用途 |
|---|---|
| **Live** | 创建/启动会话；选择控制器与 plant |
| **Replay** | 回放实验（无需 CarSim） |
| **Edit** | 编辑赛道/场景几何与 NPC |
| **Assets** | 列出 / 打开 / 删除 track 与 scenario |

示例配置：`configs/tracks/`、`configs/scenarios/`。

## 命令行（Mock）

```powershell
.\.venv\Scripts\activate

dynlimit --plant mock --steering 0.05 --throttle 0.3

dynlimit run --plant mock --scenario-id web_demo_001 --duration 20 --display-name demo_01
```

结果目录：`data/experiments/<id>/`。

## 配置 / 数据 / CarSim 路径

本快照自带 `dynlimit.toml`：

```toml
[paths]
config_root = "configs"
data_root = "data"
carsim_db = "carsim_db"   # 也可写绝对路径，如 "D:/Carsim/db"
```

覆盖优先级（高→低）：

1. CLI：`--config-root` / `--data-root` / `--carsim-db` / `--toml`
2. 环境变量：`DYNLIMIT_CONFIG_ROOT`、`DYNLIMIT_DATA_ROOT`、`DYNLIMIT_CARSIM_DB`、`DYNLIMIT_TOML`
3. `dynlimit.toml` 的 `[paths]`
4. `./configs`、`./data`、`./carsim_db`

`GET /api/health` 会回显解析后的根路径。API 端口：`DYNLIMIT_API_PORT`（默认 `8000`）。前端代理：`DYNLIMIT_API_PROXY`。

## 可选：CarSim

本快照**不包含** CarSim 数据库或 `pycarsimlib`。

若你已有合法 CarSim 环境：

1. 在 `dynlimit.toml` 中设置 `carsim_db`（需含 `simfile.sim`）。
2. 保证可 `import pycarsimlib`（自行安装或放置）。
3. 使用 `--plant carsim` 或 API `"plant": "carsim"`。

CarSim 等第三方许可与分发条款由使用者自行遵守。

## 许可证

MIT — 见 [LICENSE](LICENSE)。

## 免责声明

本软件面向研究与教学，不提供任何担保。第三方软件（含 CarSim）的许可合规由使用者自行负责。
