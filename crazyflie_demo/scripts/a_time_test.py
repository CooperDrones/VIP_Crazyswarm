from datetime import datetime
import time

at_start = datetime.utcnow()
print(at_start.second)

time.sleep(3)

at_later = datetime.utcnow()
print(at_later.second)