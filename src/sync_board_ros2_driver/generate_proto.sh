#!/bin/bash
set -e

# -------------------------------
# 配置
# -------------------------------
PROTO_DIR="./proto"
CPP_OUT_DIR="./proto_generated/cpp"
PYTHON_OUT_DIR="./proto_generated/python"

# protoc 可执行文件路径（默认使用系统 PATH）
PROTOC=$(which protoc)
if [ -z "$PROTOC" ]; then
    echo "Error: protoc not found. Please install protobuf compiler."
    exit 1
fi

# -------------------------------
# 创建输出目录
# -------------------------------
mkdir -p "${CPP_OUT_DIR}"
mkdir -p "${PYTHON_OUT_DIR}"

# -------------------------------
# 遍历 proto 文件
# -------------------------------
for proto_file in "${PROTO_DIR}"/*.proto; do
    proto_name=$(basename "$proto_file" .proto)
    echo "Generating C++ and Python for $proto_name.proto"

    # C++ 输出
    $PROTOC --cpp_out="${CPP_OUT_DIR}" --proto_path="${PROTO_DIR}" "$proto_file"

    # Python 输出
    $PROTOC --python_out="${PYTHON_OUT_DIR}" --proto_path="${PROTO_DIR}" "$proto_file"
done

echo "---------------------------"
echo "Protobuf generation completed!"
echo "C++ files in: ${CPP_OUT_DIR}"
echo "Python files in: ${PYTHON_OUT_DIR}"

# 复制生成的文件到 sync_client 目录
SYNC_CLIENT_DIR="./sync_client"
rm -rf "${SYNC_CLIENT_DIR}/proto_generated/"
mkdir -p "${SYNC_CLIENT_DIR}/proto_generated/"
cp -r "${CPP_OUT_DIR}/" "${SYNC_CLIENT_DIR}/proto_generated/cpp/"
# cp -r "${PYTHON_OUT_DIR}/" "${SYNC_CLIENT_DIR}/proto_generated/python/"

echo "Copied generated files to: ${SYNC_CLIENT_DIR}/proto_generated/"
