use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use zenoh::prelude::r#async::*;
use crate::boids::Boid;
use bincode;

pub struct ZenohManager {
    session: Session,
    pub neighbors: Arc<Mutex<HashMap<String, Boid>>>,
    drone_id: String,
}

impl ZenohManager {
    pub async fn new(drone_id: String) -> Self {
        let session = zenoh::open(config::default()).res().await.unwrap();
        let neighbors = Arc::new(Mutex::new(HashMap::new()));

        let neighbors_clone = Arc::clone(&neighbors);
        let my_id = drone_id.clone();

        let subscriber = session
            .declare_subscriber("swarm/*/state")
            .res()
            .await
            .unwrap();

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                let key = sample.key_expr.as_str();
                let parts: Vec<&str> = key.split('/').collect();
                if parts.len() >= 2 {
                    let sender_id = parts[1].to_string();
                    if sender_id != my_id {
                        if let Ok(boid) = bincode::deserialize::<Boid>(&sample.payload.contiguous()) {
                            let mut n = neighbors_clone.lock().unwrap();
                            n.insert(sender_id, boid);
                        }
                    }
                }
            }
        });

        ZenohManager {
            session,
            neighbors,
            drone_id,
        }
    }

    pub async fn publish_state(&self, boid: &Boid) {
        let key = format!("swarm/{}/state", self.drone_id);
        if let Ok(payload) = bincode::serialize(boid) {
            self.session.put(key, payload).res().await.unwrap();
        }
    }
}
