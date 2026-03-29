/// Concatenate any number of byte arrays or byte slices at compile time.
/// Usage:
/// ```rust
/// const A: [u8; 4] = [0, 1, 2, 3];
/// const B: [u8; 3] = [4, 5, 6];
/// const NAME: [u8; 7] = concat_any_bytes!(b"ST", A, B);
/// ```
#[macro_export]
macro_rules! concat_any_bytes {
    ($($arr:expr),+ $(,)?) => {{
        // Count total length at compile time
        const fn total_len() -> usize {
            let mut len = 0;
            $(
                len += $arr.len();
            )+
            len
        }
        const LEN: usize = total_len();

        // Concatenate into a const array
        const fn concat_all() -> [u8; LEN] {
            let mut out = [0u8; LEN];
            let mut i = 0;
            $(
                let mut j = 0;
                while j < $arr.len() {
                    out[i] = $arr[j];
                    i += 1;
                    j += 1;
                }
            )+
            out
        }

        concat_all()
    }};
}
