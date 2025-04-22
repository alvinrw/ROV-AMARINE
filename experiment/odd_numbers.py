import multiprocessing
import time

def print_odd_numbers():
    start_time = time.time()  
    num = 1
    while True:
        print(num)
        num += 2
        time.sleep(0.5)  # Delay of 0.5 seconds
    # We'll never reach the processing time line due to the infinite loop

if __name__ == "__main__":
    process = multiprocessing.Process(target=print_odd_numbers)
    process.start()
