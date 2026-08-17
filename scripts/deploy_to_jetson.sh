#!/usr/bin/env bash
# 部署脚本: 把项目从 Windows 开发机同步到 Jetson Nano (192.168.123.164)
# 用法 (Windows PowerShell / Git Bash):
#   bash scripts/deploy_to_jetson.sh
set -euo pipefail

# 1) 配置
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_IP="${ROBOT_IP:-192.168.123.164}"
ROBOT_DIR="${ROBOT_DIR:-~/unitreeR1}"
LOCAL_DIR="${LOCAL_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"

echo "==> 部署 $LOCAL_DIR"
echo "    目标: ${ROBOT_USER}@${ROBOT_IP}:${ROBOT_DIR}"

# 2) 同步 (排除 venv / cache / 日志)
rsync -avz --delete \
  --exclude '__pycache__' \
  --exclude '.git' \
  --exclude 'venv' \
  --exclude '.venv' \
  --exclude 'logs/*.log' \
  --exclude 'deploy/' \
  -e ssh \
  "$LOCAL_DIR/" \
  "${ROBOT_USER}@${ROBOT_IP}:${ROBOT_DIR}/"

echo "==> 同步完成"

# 3) 在机器人上安装依赖
ssh "${ROBOT_USER}@${ROBOT_IP}" <<EOF
set -e
cd ${ROBOT_DIR}
echo "==> Python 版本:"
python3 --version

echo "==> 升级 pip:"
python3 -m pip install --upgrade pip

echo "==> 安装 Python 依赖:"
python3 -m pip install -r requirements.txt
EOF

echo "==> 部署成功! SSH 登录后即可运行测试:"
echo "    ssh ${ROBOT_USER}@${ROBOT_IP}"
echo "    cd ${ROBOT_DIR}"
echo "    python3 scripts/test_sdk_connection.py eth0"
