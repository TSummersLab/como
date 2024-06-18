from datetime import datetime

def generate_filename_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%Y-%m-%d_%H:%M:%S")
    return formatted_timestamp

def generate_current_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%M:%S.%f")
    return formatted_timestamp

def time_difference(t1, t2):
    date_format = "%M:%S.%f"
    prev = datetime.strptime(t1, date_format)
    now = datetime.strptime(t2, date_format)
    return (now - prev).total_seconds() * 1000

def find_operation_time(t1, t2):
    t1 = t2
    t2 = generate_cur_timestamp()
    return t1, t2, time_difference(t1, t2)

