pub trait Flash {
    fn write<const BYTES: usize>(&mut self, bytes: &[u8; BYTES]) -> u32;
}
