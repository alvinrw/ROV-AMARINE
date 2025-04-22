import multiprocessing
import even_numbers
import odd_numbers

if __name__ == "__main__":
    even_process = multiprocessing.Process(target=even_numbers.print_even_numbers)
    odd_process = multiprocessing.Process(target=odd_numbers.print_odd_numbers)

    even_process.start()
    odd_process.start()

    even_process.join()
    odd_process.join()
