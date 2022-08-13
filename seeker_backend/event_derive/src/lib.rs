use proc_macro::TokenStream;
use quote::quote;
use syn;

#[proc_macro_derive(WriteableEventData)]
pub fn event_macro_derive(input: TokenStream) -> TokenStream {
    // Construct a representation of Rust code as a syntax tree
    // that we can manipulate
    let ast = syn::parse(input).unwrap();

    // Build the trait implementation
    impl_event_macro(&ast)
}

fn impl_event_macro(ast: &syn::DeriveInput) -> TokenStream {
    let name = &ast.ident;
    let gen = quote! {
        // #[typetag::serde]
        impl EventData for #name {}
        impl WriteableEventData for #name {}
    };
    gen.into()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
