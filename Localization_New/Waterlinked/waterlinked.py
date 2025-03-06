import queue
import requests
import threading
import time

class WaterLinked:
    def __init__(self, base_url, poll_interval=1.0, test_mode=True):
        """
        Initializes the WaterLinked instance.

        Parameters:
        - base_url: URL of the WaterLinked system ("http://192.168.7.1")
        - poll_interval: How often (in seconds) to fetch new data.
        - test_mode: If True, dummy data is returned instead of making HTTP calls.
        """
        self.base_url = base_url
        self.poll_interval = poll_interval
        self.test_mode = test_mode
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
                self.latest_data = data
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

    def get_latest_position(self):
        """
        Returns the latest position as a formatted string 'x, y, z'.
        If no data is available yet, returns an empty string.
        """
        if self.latest_data:
            return f"{self.latest_data.get('x')}, {self.latest_data.get('y')}, {self.latest_data.get('z')}"
        else:
            return ""

    def stop(self):
        """Stops the background thread gracefully."""
        self.running = False
        if self.thread:
            self.thread.join()


# Create a single instance of WaterLinked for use in the communications module.
# The communications code only needs to call waterlinked_instance.get_latest_position_string().
waterlinked_instance = WaterLinked(base_url="http://192.168.7.1", poll_interval=1.0, test_mode=True)
print(waterlinked_instance.get_latest_position())
