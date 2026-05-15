import os
import tkinter as tk
from tkinter import messagebox, ttk

import roboflow

# if "ROBOFLOW_API_KEY" not in os.environ:
#     raise ValueError("ROBOFLOW_API_KEY environment variable is not set.")
# ROBOFLOW_API_KEY = os.environ["ROBOFLOW_API_KEY"]

UPLOAD_INTERVAL = 10 # upload every nth frame
PATH_TO_FRAME_LOGS = "/home/ws/frame_logs"
PROJ_NAME = "boat-buoy-bbox"

def get_runs() -> list[str]:
    with os.scandir(PATH_TO_FRAME_LOGS) as entries:
        subdirs = []
        for entry in entries:
            if entry.is_dir():
                num_images = len(os.listdir(f"{PATH_TO_FRAME_LOGS}/{entry.name}/frames"))
                subdirs.append(f"{entry.name} {num_images} images")
    return subdirs

def get_inputs() -> dict[str, str]:
    input_data = {}
    def save_data() -> None:
        key = key_entry.get()
        folder = folder_entry.get()
        job_name = job_entry.get()
        labeler_email = labeler_entry.get()
        reviewer_email = reviewer_entry.get()
        if not key or not folder or not job_name or not labeler_email or not reviewer_email:
            messagebox.showwarning("Input Error", "Please fill in all fields")
            return
        root.destroy()
        input_data.update({
            "api_key": key,
            "folder": folder.split(' ')[0], # get the folder name without the number of images
            "job_name": job_name,
            "labeler_email": labeler_email,
            "reviewer_email": reviewer_email,
            "num_images": int(folder.split(' ')[1]) # get the number of images
        })
    
    root = tk.Tk()
    root.title("Roboflow input")

    # Labels + input fields
    # TODO, make user not enter api_key every time.
    # Alternatives: environment variable, roboflow login
    tk.Label(root, text="API Key:").grid(row=0, column=0)
    key_entry = tk.Entry(root)
    key_entry.grid(row=0, column=1)

    tk.Label(root, text="Folder select:").grid(row=1, column=0)
    folder_entry = ttk.Combobox(root, values=get_runs(), state='readonly')
    folder_entry.grid(row=1, column=1)

    # Blank line
    tk.Label(root, text="").grid(row=2, column=0)

    tk.Label(root, text="Job name:").grid(row=3, column=0)
    job_entry = tk.Entry(root)
    job_entry.grid(row=3, column=1)

    tk.Label(root, text="Labeler email").grid(row=4, column=0)
    labeler_entry = tk.Entry(root)
    labeler_entry.grid(row=4, column=1)

    tk.Label(root, text="Reviewer email").grid(row=5, column=0)
    reviewer_entry = tk.Entry(root)
    reviewer_entry.grid(row=5, column=1)

    tk.Button(root, text="Submit", command=save_data).grid(row=6, columnspan=2)
    root.mainloop()
    return input_data

def upload_images(proj: roboflow.Project, inputs: dict[str, str]) -> None:
    folder = inputs["folder"]
    batch_name = f"{folder}_batch"
    images_uploaded = 0
    for i in range(0, inputs["num_images"], UPLOAD_INTERVAL):
        image_path = f"{PATH_TO_FRAME_LOGS}/{folder}/frames/frame{i:04d}.png"
        if os.path.exists(image_path):
            proj.upload(image_path, batch_name=batch_name)
            images_uploaded += 1
        else:
            print(f"Image {image_path} does not exist, skipping.")
    batches = proj.get_batches()
    batch_id = ''
    for batch in batches['batches']:
        if batch['name'] == batch_name:
            batch_id = batch['id']
            break
    if batch_id == '':
        raise ValueError(f"Batch with name {batch_name} not found.")
    proj.create_annotation_job(inputs["job_name"], batch_id, images_uploaded,
                               inputs["labeler_email"], inputs["reviewer_email"])

def main() -> int:
    if not os.path.exists(PATH_TO_FRAME_LOGS):
        messagebox.showinfo("Warning", f"Could not find any folder {PATH_TO_FRAME_LOGS}")
        return
    inputs = get_inputs()
    
    rf = roboflow.Roboflow(api_key=inputs["api_key"])
    proj = rf.workspace().project(PROJ_NAME)
    upload_images(proj, inputs)

if __name__ == "__main__":
    main()
