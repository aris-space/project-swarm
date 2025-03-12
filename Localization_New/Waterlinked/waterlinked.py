import queue
import requests
import threading
import time

class WaterLinked:
    def __init__(self, base_url, poll_interval=1.0, test_mode=False):
        """
        Initializes the WaterLinked instance.

        Parameters:
        - base_url: URL of the WaterLinked system (if connected to swarm: "http://192.168.8.108", if connected to UnderwaterGPS: "http://192.168.2.94")
        - poll_interval: How often (in seconds) to fetch new data.
        - test_mode: If True, dummy data is returned instead of making HTTP calls.
        """
        self.base_url = base_url
        self.poll_interval = poll_interval
        self.test_mode = test_mode
        self.data_condition = threading.Condition() # used to signal when new data is available 
        self.latest_data = None  # Holds the most recent data (a dict)
        self.running = True
        # Create and automatically start the background thread
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        """Background thread function that continuously fetches and stores data."""
        while self.running:
            data = self._fetch_data()
            if data:
                # signal that new data is available
                with self.data_condition:
                    self.latest_data = data
                    self.data_condition.notify_all()                
            time.sleep(self.poll_interval)

    def _fetch_data(self):
        """
        Fetches the latest acoustic position data from the WaterLinked API.
        In test mode, it returns dummy data.
        """
        endpoint = f"{self.base_url}/api/v1/position/acoustic/filtered"
        if self.test_mode:
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

    def get_latest_position(self, timeout=None):
        """
        Returns the latest position as a formatted string 'x, y, z'.
        If no data is available yet, returns an empty string.
        """
        with self.data_condition:
            if self.latest_data is None:
                self.data_condition.wait(timeout=timeout) # wait until new data is available
            if self.latest_data:
                x = round(self.latest_data.get('x',0),2)
                y = round(self.latest_data.get('y',0),2)
                z = round(self.latest_data.get('z',0),2)

                brim_size = 0.3 # distance from wall
                if x < brim_size:
                    x = brim_size
                elif x > 5 - brim_size:
                    x = 5 - brim_size
                if y < brim_size:
                    y = brim_size
                elif y > 2.5 - brim_size:
                    y = 2.5 - brim_size

                return [x, y, z]
            else:
                return ""

    def stop(self):
        """Stops the background thread gracefully."""
        self.running = False
        if self.thread:
            self.thread.join()
