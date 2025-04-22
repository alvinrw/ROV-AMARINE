import ultralytics
import torch

def check_versions():
    # Check Ultralytics version
    print(f"Ultralytics version: {ultralytics.__version__}")

    # Check PyTorch version
    print(f"PyTorch version: {torch.__version__}")

    # Check if CUDA is available with GPU support
    if torch.cuda.is_available():
        print("CUDA is available with GPU support.")
        print(f"Number of available GPUs: {torch.cuda.device_count()}")
        print(f"Current GPU device: {torch.cuda.get_device_name(0)}")
    else:
        print("CUDA is not available or GPU support is not enabled.")

if __name__ == "__main__":
    check_versions()