from flask import Flask, render_template, request, jsonify
from threading import Thread
import time, datetime, uuid, requests

app = Flask(__name__)

# Global variables untuk menyimpan data Cat Feeder
cat_feeder_data = {
    'ultrasonic': 0,
    'servo_active': False,
    'feeding_count': 0,
    'timestamp': 'No data',
    'last_update': None
}

# Command queue untuk servo
servo_command = {'command': ''}

# Feeding history
feeding_history = []

# Simpan jadwal di memori (untuk contoh, gunakan database untuk produksi)
feed_schedules = []

@app.route('/')
def index():
    return render_template('index.html')

# ===== CAT FEEDER ENDPOINTS =====
@app.route('/esp32/sensor', methods=['POST'])
def receive_sensor_data():
    """Menerima data sensor dari ESP32 Cat Feeder"""
    try:
        data = request.get_json()
        cat_feeder_data.update({
            'ultrasonic': data.get('ultrasonic', 0),
            'servo_active': data.get('servo_active', False),
            'feeding_count': data.get('feeding_count', 0),
            'timestamp': data.get('timestamp', 'No timestamp'),
            'last_update': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })
        print(f"Received cat feeder data: Distance={data.get('ultrasonic')}cm, Servo={data.get('servo_active')}, Count={data.get('feeding_count')}")
        return jsonify({'status': 'success', 'message': 'Data received'}), 200
    except Exception as e:
        print(f"Error receiving sensor data: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/esp32/servo', methods=['GET'])
def get_servo_command():
    """ESP32 Cat Feeder mengecek command"""
    global servo_command
    command = servo_command['command']
    # Reset command setelah dikirim
    servo_command['command'] = ''
    return jsonify({'command': command})

@app.route('/control_servo', methods=['POST'])
def control_servo():
    """Menerima log feeding dari ESP32"""
    try:
        data = request.get_json()
        action = data.get('action', '')
        source = data.get('source', 'unknown')
        distance_before = data.get('distance_before', 'N/A')
        timestamp = data.get('timestamp', datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        
        # Simpan ke feeding history
        feeding_entry = {
            'action': action,
            'source': source,
            'distance_before': distance_before,
            'timestamp': timestamp,
            'server_time': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        feeding_history.append(feeding_entry)
        
        # Keep only last 50 entries
        if len(feeding_history) > 50:
            feeding_history.pop(0)
        
        print(f"Feeding logged: {action} from {source} at {timestamp}")
        return jsonify({'status': 'success', 'message': 'Feeding logged'}), 200
    except Exception as e:
        print(f"Error logging feeding: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

# ===== WEB INTERFACE API ENDPOINTS =====
@app.route('/api/cat-feeder-data')
def api_cat_feeder_data():
    """API untuk mendapatkan data cat feeder"""
    return jsonify(cat_feeder_data)

@app.route('/api/feed-cat', methods=['POST'])
def api_feed_cat():
    """API untuk memberi makan kucing dari web interface"""
    global servo_command
    servo_command['command'] = 'feed'
    print("Feed command sent from web interface")
    return jsonify({'status': 'success', 'message': 'Feed command sent to ESP32'})

@app.route('/api/feeding-history')
def api_feeding_history():
    """API untuk mendapatkan riwayat feeding"""
    # Return last 10 entries, newest first
    return jsonify(feeding_history[-10:][::-1])

@app.route('/api/system-status')
def api_system_status():
    """API untuk mendapatkan status sistem"""
    # Check if ESP32 is online (last update within 10 seconds)
    last_update = cat_feeder_data.get('last_update')
    is_online = False
    if last_update:
        try:
            last_time = datetime.datetime.strptime(last_update, '%Y-%m-%d %H:%M:%S')
            time_diff = (datetime.datetime.now() - last_time).total_seconds()
            is_online = time_diff < 15  # Consider online if updated within 15 seconds
        except:
            is_online = False
    
    return jsonify({
        'esp32_online': is_online,
        'total_feedings': len(feeding_history),
        'server_time': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    })

@app.route('/api/feed-schedule', methods=['GET'])
def get_feed_schedule():
    """API untuk mendapatkan jadwal pemberian makan"""
    return jsonify(feed_schedules)

@app.route('/api/feed-schedule', methods=['POST'])
def add_feed_schedule():
    """API untuk menambah jadwal pemberian makan"""
    data = request.get_json()
    time = data.get('time')
    if not time:
        return jsonify({'message': 'Waktu tidak boleh kosong'}), 400
    # Cek duplikat
    for s in feed_schedules:
        if s['time'] == time:
            return jsonify({'message': 'Jadwal sudah ada'}), 400
    new_schedule = {'id': str(uuid.uuid4()), 'time': time}
    feed_schedules.append(new_schedule)
    return jsonify(new_schedule), 201

@app.route('/api/feed-schedule/<id>', methods=['DELETE'])
def delete_feed_schedule(id):
    """API untuk menghapus jadwal pemberian makan"""
    global feed_schedules
    feed_schedules = [s for s in feed_schedules if s['id'] != id]
    return '', 204

def trigger_feeding():
    # Panggil fungsi untuk menggerakkan servo/ESP32 di sini
    print("Feeding triggered by schedule!")

def scheduler_loop():
    last_triggered = set()
    while True:
        now = datetime.datetime.now().strftime('%H:%M')
        for sched in feed_schedules:
            if sched['time'] == now and sched['id'] not in last_triggered:
                try:
                    # Kirim perintah feeding sama seperti tombol manual
                    requests.post('http://localhost:5000/api/feed-cat')
                    print(f"Feeding triggered by schedule at {now}")
                except Exception as e:
                    print("Failed to trigger feeding:", e)
                last_triggered.add(sched['id'])
        # Reset last_triggered setiap menit
        if datetime.datetime.now().second == 0:
            last_triggered.clear()
        time.sleep(1)

# Jalankan scheduler di background
Thread(target=scheduler_loop, daemon=True).start()

if __name__ == '__main__':
    print("=== ESP32 Cat Feeder Server ===")
    print("ESP32 Endpoints:")
    print("  POST /esp32/sensor - Receive sensor data")
    print("  GET  /esp32/servo - Get servo commands") 
    print("  POST /control_servo - Log feeding actions")
    print()
    print("Web Interface:")
    print("  http://localhost:5000 - Dashboard")
    print()
    
    app.run(host='0.0.0.0', port=5000, debug=True)