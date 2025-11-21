import sys

if len(sys.argv) < 2:
    print("Usage: python test.py <onnx_file>")
    sys.exit(1)

if not sys.argv[1].endswith('.pt.onnx'):
    print("Error: ONNX file must end with .pt.onnx")
    sys.exit(1)

CONFIG_FILE = 'config_infer_primary_yolo11.txt'
onnx_file = sys.argv[1]

with open(CONFIG_FILE, 'r') as file:
    content = file.read()
    split_content = content.split('\n\n')
    onnx_section = split_content[1]
    engine_section = split_content[2]
    onnx_lines = onnx_section.split('\n')
    engine_lines = engine_section.split('\n')

    for i in range(len(onnx_lines)):
        if onnx_lines[i].startswith('onnx-file='):
            onnx_lines[i] = "#" + onnx_lines[i]
    onnx_lines.append(f"onnx-file={onnx_file}")

    for i in range(len(engine_lines)):
        if engine_lines[i].startswith('model-engine-file='):
            engine_lines[i] = "#" + engine_lines[i]
    engine_lines.append(f"model-engine-file={onnx_file.replace('.pt.onnx', '_model_b1_gpu0_fp16.engine')}")

    onnx_content = "\n".join(onnx_lines)
    engine_content = "\n".join(engine_lines)

    split_content[1] = onnx_content
    split_content[2] = engine_content

    content = "\n\n".join(split_content)

with open(CONFIG_FILE, 'w') as file:
    file.write(content)