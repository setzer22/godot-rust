use crate::core_types::PoolArray;

/// A reference-counted vector of `i32` that uses Godot's pool allocator.
pub type Int32Array = PoolArray<i32>;

godot_test!(
    test_int32_array_access {
        use crate::object::NewRef as _;

        let arr = (0..8).collect::<Int32Array>();

        let original_read = {
            let read = arr.read();
            assert_eq!(&[0, 1, 2, 3, 4, 5, 6, 7], read.as_slice());
            read.clone()
        };

        let mut cow_arr = arr.new_ref();

        {
            let mut write = cow_arr.write();
            assert_eq!(8, write.len());
            for i in write.as_mut_slice() {
                *i *= 2;
            }
        }

        for i in 0..8 {
            assert_eq!(i * 2, cow_arr.get(i as i32));
        }

        // the write shouldn't have affected the original array
        assert_eq!(&[0, 1, 2, 3, 4, 5, 6, 7], original_read.as_slice());
    }
);

godot_test!(
    test_int32_array_debug {
        let arr = (0..8).collect::<Int32Array>();
        assert_eq!(format!("{:?}", arr), "[0, 1, 2, 3, 4, 5, 6, 7]");
    }
);
