import argparse
import csv
import datetime
import queue
import requests
import threading
import time

class DataProducer(threading.Thread):
    def __init__(self, base_url, poll_interval, test_mode, data_queue):
        super().__init__(daemon=True)
        self.base_url = base_url
        self.poll_interval = poll_interval
        self.test_mode = test_mode
        self.data_queue = data_queue
        self.running = True

    def run(self):
        while self.running:
            data = self.fetch_data()
            if data is not None:
                try:
                    self.data_queue.put(data, timeout=1)
                    #print(f"Data buffered: {data}")
                except queue.Full:
                    print("Buffer full, skipping data point.")
            time.sleep(self.poll_interval)

    def fetch_data(self):
        endpoint = f"{self.base_url}/api/v1/position/acoustic/filtered"
        if self.test_mode:
            # Return default values when in test mode
            return {"x": 1.23, "y": 4.56, "z": 7.89}
        try:
            response = requests.get(endpoint)
        except requests.exceptions.RequestException as exc:
            print("Exception occurred:", exc)
            return None

        if response.status_code != requests.codes.ok:
            print("Got error {}: {}".format(response.status_code, response.text))
            return None

        return response.json()

    def stop(self):
        self.running = False


class DataConsumer(threading.Thread):
    def __init__(self, csv_filename, data_queue):
        super().__init__(daemon=True)
        self.csv_filename = csv_filename
        self.data_queue = data_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.data_queue.get(timeout=1)
                self.log_to_csv(data)
                # Print the acoustic x, y, and z positions
                print(f"Acoustic Position -> X: {data.get('x')}, Y: {data.get('y')}, Z: {data.get('z')}")
                #print(f"Data processed: {data}")
                self.data_queue.task_done()
            except queue.Empty:
                continue

    def log_to_csv(self, data):
        fieldnames = ['timestamp', 'x', 'y', 'z']
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            csvfile.seek(0, 2)  # Move to the end of the file
            if csvfile.tell() == 0:
                writer.writeheader()
            writer.writerow({
                'timestamp': datetime.datetime.now().isoformat(),
                'x': data.get('x'),
                'y': data.get('y'),
                'z': data.get('z')
            })

    def stop(self):
        self.running = False


def main():
    parser = argparse.ArgumentParser(description="WaterLinked Data Buffer with Terminal Output")
    parser.add_argument("-u", "--url", type=str, default="https://demo.waterlinked.com",
                        help="Base URL to use")
    parser.add_argument("-t", "--test", action="store_true",
                        help="Enable test mode with default values")
    args = parser.parse_args()

    base_url = args.url
    test_mode = args.test
    csv_filename = "positions.csv"

    # Create a FIFO queue for buffering data
    data_queue = queue.Queue(maxsize=100)

    # Instantiate producer and consumer threads
    producer = DataProducer(base_url, poll_interval=1.0, test_mode=test_mode, data_queue=data_queue)
    consumer = DataConsumer(csv_filename, data_queue=data_queue)

    # Start the threads
    producer.start()
    consumer.start()

    # Keep the main thread alive until interrupted
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down.")
        producer.stop()
        consumer.stop()
        producer.join()
        consumer.join()


if __name__ == "__main__":
    main()