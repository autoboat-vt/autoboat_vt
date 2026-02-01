import sys

if len(sys.argv) < 3:
    print("Usage: python modify_config_file.py <name_of_model> <yolo_version>")
    sys.exit(1)

# if not sys.argv[1].endswith('.pt.onnx'):
#     print("Error: ONNX file must end with .pt.onnx")
#     sys.exit(1)

BATCH_SIZE = 2
FP_VER = 16

CONFIG_FILE = f'config_infer_primary_yolo{sys.argv[2]}.txt'
model_name = sys.argv[1]

with open(CONFIG_FILE, 'r') as file:
    content = file.read()
    split_content = content.split('\n\n')
    onnx_section = split_content[1]
    engine_section = split_content[2]
    labels_section = split_content[3]
    onnx_lines = onnx_section.split('\n')
    engine_lines = engine_section.split('\n')
    labels_lines = labels_section.split('\n')

    found_onnx_entry = False
    for i in range(len(onnx_lines)):
        if onnx_lines[i].startswith('onnx-file='):
            onnx_lines[i] = "#" + onnx_lines[i]
        if onnx_lines[i] == f"#onnx-file=./onnx_files/{model_name}.pt.onnx":
            onnx_lines[i] = onnx_lines[i][1:] # uncomment line so the model can be used
            found_onnx_entry = True
    if not found_onnx_entry:
        onnx_lines.append(f"onnx-file=./onnx_files/{model_name}.pt.onnx")

    found_engine_entry = False
    for i in range(len(engine_lines)):
        if engine_lines[i].startswith('model-engine-file='):
            engine_lines[i] = "#" + engine_lines[i]
        if engine_lines[i] == f"#model-engine-file=./engine_files/{model_name}_model_b{BATCH_SIZE}_gpu0_fp{FP_VER}.engine":
            engine_lines[i] = engine_lines[i][1:] # uncomment line so the model can be used
            found_engine_entry = True
    if not found_engine_entry:
        engine_lines.append(f"model-engine-file=./engine_files/{model_name}_model_b{BATCH_SIZE}_gpu0_fp{FP_VER}.engine")
    found_labels_entry = False
    for i in range(len(labels_lines)):
        if labels_lines[i].startswith('labelfile-path='):
            labels_lines[i] = "#" + labels_lines[i]
        if labels_lines[i] == f"#labelfile-path=./label_files/{model_name}_labels.txt":
            labels_lines[i] = labels_lines[i][1:] # uncomment line so the model can be used
            found_labels_entry = True
    if not found_labels_entry:
        labels_lines.append(f"labelfile-path=./label_files/{model_name}_labels.txt")
    
    onnx_content = "\n".join(onnx_lines)
    engine_content = "\n".join(engine_lines)
    labels_content = "\n".join(labels_lines)

    split_content[1] = onnx_content
    split_content[2] = engine_content
    split_content[3] = labels_content

    content = "\n\n".join(split_content)

with open(CONFIG_FILE, 'w') as file:
    file.write(content)