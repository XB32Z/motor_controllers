// From
// https://stackoverflow.com/questions/27693812/how-to-handle-unique-ptrs-with-swig
namespace std {
  %feature("novaluewrapper") unique_ptr;
  template <typename Type, void(*deleter)(Type*)>
  struct unique_ptr {
     typedef Type* pointer;

     explicit unique_ptr( pointer Ptr );
     unique_ptr (unique_ptr&& Right);
     template<class Type2, Class Del2> unique_ptr( unique_ptr<Type2, Del2>&& Right );
     unique_ptr( const unique_ptr& Right) = delete;


     pointer operator-> () const;
     /*pointer release ();
     void reset (pointer __p=pointer());
     void swap (unique_ptr &__u);
     pointer get () const;*/
     operator bool () const;

     ~unique_ptr();
  };
}

%define wrap_unique_ptr(Name, Type, Deleter)
  %template(Name) std::unique_ptr<Type, Deleter>;
  %newobject std::unique_ptr<Type, Deleter>::release;

  %typemap(out) std::unique_ptr<Type, Deleter> %{
    $result = SWIG_NewPointerObj((new $1_ltype(std::move($1))), $&1_descriptor, SWIG_POINTER_OWN);
  %}

  %typemap(in) std::unique_ptr<Type, Deleter> %{
    $1 = std::move($input);
  %}

%enddef