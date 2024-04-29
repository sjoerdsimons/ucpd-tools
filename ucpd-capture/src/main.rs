use analyzer_cbor::{AnalyserData, Decoder};
use byteorder::{ByteOrder, LittleEndian};
use clap::Parser;
use std::{
    fs::File,
    io::{Read, Write},
    path::PathBuf,
    time::Duration,
};
use usb_pd::{header::Header, message::Message};

#[derive(Parser, Debug)]
struct Opts {
    #[clap(short, long)]
    output: Option<PathBuf>,
    #[clap(short, long)]
    from_file: bool,
    input: PathBuf,
}

fn do_work<R: Read, W: Write>(mut input: R, mut output: Option<W>) {
    let mut data = [0u8; 1024];
    let mut d = Decoder::new(&mut data);
    loop {
        let read_buffer = d.write_buffer();

        let r = input.read(read_buffer).unwrap();
        if r == 0 {
            return;
        }
        if let Some(ref mut output) = output {
            output
                .write_all(&read_buffer[0..r])
                .expect("Failed to write to output");
        }
        d.written(r);

        while let Some(data) = d.decode() {
            match data {
                AnalyserData::Version { version } => println!("Analyser version: {version}"),
                AnalyserData::PdSpy { data } => {
                    let header = Header(LittleEndian::read_u16(&data[..2]));
                    println!("======== PD Msg: {:?} ============", header.message_type());
                    println!("Header: {:#?}", header);
                    let message = Message::parse(header, &data[2..]);
                    println!("Message: {:#?}", message);
                }
                AnalyserData::Voltage { label, value } => {
                    println!("== {label}: {value}V ==");
                }
            }
        }
    }
}

fn main() {
    let opts = Opts::parse();

    let output = opts
        .output
        .map(|o| File::create(o).expect("Failed to open output file"));

    if opts.from_file {
        let input = File::open(opts.input).expect("Failed to open input file");
        do_work(input, output);
    } else {
        let port = serialport::new(opts.input.to_str().unwrap(), 115200)
            .timeout(Duration::from_secs(u32::MAX as u64))
            .open()
            .expect("Failed to open serial port");
        do_work(port, output);
    }
}
