# /Localization_New/aa_localization_methods.py

def waterlinked_method1(localization_sensor, swarm_storage, drone_id, logger):
    """
    Fetches the latest localization data from the Waterlinked sensor and updates the swarm storage.

    Args:
        localization_sensor: The sensor object to fetch localization data.
        swarm_storage: The data storage object for swarm information.
        drone_id: The ID of the current drone.
        logger: Logging function.
    """
    try:
        # Get localization data (may block up to 2 seconds)
        localization = localization_sensor.get_latest_position(timeout=2)
        
        if localization is None:
            logger("Waterlinked: No new data")
        else:
            swarm_storage.raw_drones[drone_id].update_position(values=localization)
            logger(f"Updated waterlinked: {localization}")
    
    except Exception as e:
        logger(f"Error in waterlinked_method1: {e}")
