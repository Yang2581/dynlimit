import { Component, type ErrorInfo, type ReactNode } from "react";

type Props = { children: ReactNode };
type State = { error: Error | null };

export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  componentDidCatch(error: Error, info: ErrorInfo): void {
    console.error("UI crashed", error, info);
  }

  render() {
    if (this.state.error) {
      return (
        <div style={{ padding: "2rem", color: "#e8f0ea", fontFamily: "sans-serif" }}>
          <h1>DynLimit UI error</h1>
          <pre style={{ whiteSpace: "pre-wrap", color: "#c45c4a" }}>
            {this.state.error.message}
          </pre>
          <button type="button" onClick={() => this.setState({ error: null })}>
            Try again
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}
