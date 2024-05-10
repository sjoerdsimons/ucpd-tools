use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::pubsub::{self, DynImmediatePublisher, DynSubscriber, PubSubChannel};

/// Watch like channel with 1 publisher and multiple subscribers
pub struct Watch<M: RawMutex, T: Clone, const SUBS: usize>(PubSubChannel<M, T, 1, SUBS, 1>);

impl<M: RawMutex, T: Clone, const SUBS: usize> Watch<M, T, SUBS> {
    pub const fn new() -> Self {
        Watch(PubSubChannel::new())
    }

    pub fn subscribe(&self) -> Result<WatchSubscriber<T>, pubsub::Error> {
        self.0
            .dyn_subscriber()
            .map(|sub| WatchSubscriber { sub, last: None })
    }

    pub fn publish(&self) -> WatchPublisher<T> {
        WatchPublisher(self.0.dyn_immediate_publisher())
    }
}

pub struct WatchSubscriber<'a, T: Clone> {
    sub: DynSubscriber<'a, T>,
    last: Option<T>,
}

impl<'a, T: Clone> WatchSubscriber<'a, T> {
    pub fn last(&mut self) -> Option<&T> {
        if let Some(l) = self.sub.try_next_message_pure() {
            self.last = Some(l);
        }
        self.last.as_ref()
    }

    pub async fn wait_next(&mut self) -> &T {
        self.last = Some(self.sub.next_message_pure().await);
        self.last.as_ref().unwrap()
    }
}

pub struct WatchPublisher<'a, T: Clone>(DynImmediatePublisher<'a, T>);
impl<T: Clone> WatchPublisher<'_, T> {
    pub fn publish(&self, value: T) {
        self.0.publish_immediate(value)
    }
}
