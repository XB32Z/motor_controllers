// From
// https://stackoverflow.com/questions/27693812/how-to-handle-unique-ptrs-with-swig
namespace std {
  %feature("novaluewrapper") future;
  template <typename Type>
  struct future {
     explicit future();

     Type get ();

     ~future();
  };
}

%define wrap_future(Name, Type)
  %template(Name) std::future<Type>;

  %typemap(out) std::future<Type> %{
    $result = SWIG_NewPointerObj((new $1_ltype(std::move($1))), $&1_descriptor, SWIG_POINTER_OWN);
  %}

  %typemap(in) std::unique_ptr<Type> %{
    $1 = $input;
  %}

%enddef