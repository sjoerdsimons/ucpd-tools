#![cfg_attr(not(feature = "std"), no_std)]
use minicbor::{bytes::ByteSlice, Decode, Encode};

#[derive(Debug, Encode, Decode)]
pub enum AnalyserData<'a> {
    #[n(1)]
    Version {
        #[n(1)]
        version: &'a str,
    },
    #[n(2)]
    PdSpy {
        #[n(1)]
        data: &'a ByteSlice,
    },
    #[n(3)]
    Power {
        #[n(1)]
        label: &'a str,
        #[n(2)]
        voltage: f64,
        #[n(3)]
        current: Option<f64>,
    },
}

#[derive(Debug)]
pub struct Decoder<'a> {
    data: &'a mut [u8],
    end: usize,
    read: usize,
}

impl<'a> Decoder<'a> {
    pub fn new(data: &'a mut [u8]) -> Self {
        Decoder {
            data,
            read: 0,
            end: 0,
        }
    }

    pub fn decode(&mut self) -> Option<AnalyserData> {
        if self.read >= self.data.len() {
            return None;
        }

        loop {
            let mut d = minicbor::decode::Decoder::new(&self.data[self.read..self.end]);
            match d.decode() {
                Ok(v) => {
                    self.read += d.position();
                    return Some(v);
                }
                Err(e) if e.is_end_of_input() => return None,
                // TODO report error
                Err(_e) => {
                    // On error skip until parsing works again
                    self.read += 1;
                    if self.read >= self.data.len() {
                        return None;
                    }
                }
            }
        }
    }

    pub fn write_buffer(&mut self) -> &mut [u8] {
        self.data.copy_within(self.read..self.end, 0);
        self.end -= self.read;
        self.read = 0;
        &mut self.data[self.end..]
    }

    pub fn written(&mut self, written: usize) {
        self.end += written;
    }
}
