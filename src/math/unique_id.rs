use std::sync::Mutex;

static COUNT: Mutex<i32> = Mutex::new(0);

pub fn unique_id() -> String {
    let mut count = COUNT.lock().unwrap();
    *count += 1;
    return count.to_string()
}
