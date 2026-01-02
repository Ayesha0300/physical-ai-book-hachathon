import React from 'react';
import CalloutBox from './CalloutBox';
import CodeBlock from './CodeBlock';
import CollapsibleSection from './CollapsibleSection';
import GlossaryTooltip from './GlossaryTooltip';

function ModuleDetailPage({
  moduleNumber,
  title,
  description,
  chapters = [],
  technologies = [],
  requirements = {},
  progress = 0
}) {
  return (
    <div className="min-h-screen bg-dark-bg-900 text-light-text-50">
      {/* Sticky Header */}
      <header className="sticky top-0 z-10 bg-dark-bg-800 border-b border-dark-bg-700 shadow-lg">
        <div className="container mx-auto px-4 py-4">
          <div className="flex flex-col md:flex-row md:items-center md:justify-between gap-4">
            <div>
              <h1 className="text-2xl font-bold text-accent-primary-400">Module {moduleNumber}: {title}</h1>
              <p className="text-light-text-200 mt-1">{description}</p>
            </div>

            <div className="flex flex-col sm:flex-row sm:items-center gap-4">
              <div className="flex items-center gap-2">
                <span className="text-sm text-light-text-200">Progress:</span>
                <div className="w-32 bg-dark-bg-700 rounded-full h-2.5">
                  <div
                    className="bg-accent-primary-500 h-2.5 rounded-full transition-all duration-300 ease-out"
                    style={{ width: `${progress}%` }}
                  ></div>
                </div>
                <span className="text-sm text-light-text-200">{progress}%</span>
              </div>

              <div className="flex gap-2">
                <button className="px-4 py-2 bg-accent-primary-500 hover:bg-accent-primary-600 text-white rounded-lg transition-colors duration-200 text-sm">
                  Personalize
                </button>
                <button className="px-4 py-2 bg-dark-bg-700 hover:bg-dark-bg-600 text-light-text-50 rounded-lg transition-colors duration-200 text-sm border border-dark-bg-600">
                  Translate
                </button>
                <button className="px-4 py-2 bg-accent-secondary-500 hover:bg-accent-secondary-600 text-white rounded-lg transition-colors duration-200 text-sm">
                  Chatbot
                </button>
              </div>
            </div>
          </div>

          {/* Technology Badges */}
          <div className="flex flex-wrap gap-2 mt-4">
            {technologies.map((tech, index) => (
              <span
                key={index}
                className="px-3 py-1 bg-tech-ros-500 text-white rounded-full text-xs font-medium"
              >
                {tech}
              </span>
            ))}
          </div>
        </div>
      </header>

      <div className="container mx-auto px-4 py-8">
        <div className="grid grid-cols-1 lg:grid-cols-4 gap-8">
          {/* Sidebar Navigation */}
          <div className="lg:col-span-1">
            <div className="sticky top-24 bg-dark-bg-800 rounded-xl p-6 border border-dark-bg-700">
              <h3 className="font-semibold text-light-text-50 mb-4">Chapters</h3>
              <nav className="space-y-2">
                {chapters.map((chapter, index) => (
                  <a
                    key={index}
                    href={`#${chapter.id || chapter.title.toLowerCase().replace(/\s+/g, '-')}`}
                    className={`block px-3 py-2 rounded-lg text-sm transition-colors duration-200 ${
                      chapter.isCompleted
                        ? 'bg-accent-success/20 text-accent-success hover:bg-accent-success/30'
                        : 'text-light-text-200 hover:bg-dark-bg-700'
                    }`}
                  >
                    <div className="flex items-center justify-between">
                      <span>{chapter.title}</span>
                      {chapter.isCompleted && (
                        <svg className="w-4 h-4 text-accent-success" fill="currentColor" viewBox="0 0 20 20">
                          <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                        </svg>
                      )}
                    </div>
                  </a>
                ))}
              </nav>
            </div>

            {/* Hardware Requirements */}
            {requirements && (
              <div className="mt-6 bg-dark-bg-800 rounded-xl p-6 border border-dark-bg-700">
                <h3 className="font-semibold text-light-text-50 mb-4">Hardware Requirements</h3>
                <div className="space-y-3">
                  {requirements.minimum && (
                    <div>
                      <h4 className="text-sm font-medium text-accent-warning-400">Minimum</h4>
                      <p className="text-xs text-light-text-200 mt-1">{requirements.minimum}</p>
                    </div>
                  )}
                  {requirements.recommended && (
                    <div>
                      <h4 className="text-sm font-medium text-accent-success-400">Recommended</h4>
                      <p className="text-xs text-light-text-200 mt-1">{requirements.recommended}</p>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>

          {/* Main Content */}
          <div className="lg:col-span-3">
            <div className="prose prose-invert prose-lg max-w-none">
              {chapters.map((chapter, chapterIndex) => (
                <section
                  key={chapterIndex}
                  id={chapter.id || chapter.title.toLowerCase().replace(/\s+/g, '-')}
                  className="mb-12 pb-8 border-b border-dark-bg-700 last:border-b-0"
                >
                  <h2 className="text-2xl font-bold text-light-text-50 mb-4 flex items-center">
                    {chapter.title}
                    {chapter.isCompleted && (
                      <span className="ml-3 inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-accent-success/20 text-accent-success">
                        Completed
                      </span>
                    )}
                  </h2>

                  {chapter.description && (
                    <p className="text-light-text-200 mb-6">{chapter.description}</p>
                  )}

                  {/* Content sections */}
                  {chapter.content && chapter.content.map((contentItem, contentIndex) => (
                    <div key={contentIndex} className="mb-6">
                      {contentItem.type === 'text' && (
                        <p className="text-light-text-200 leading-relaxed">{contentItem.text}</p>
                      )}

                      {contentItem.type === 'code' && (
                        <CodeBlock language={contentItem.language}>
                          {contentItem.code}
                        </CodeBlock>
                      )}

                      {contentItem.type === 'callout' && (
                        <CalloutBox type={contentItem.calloutType} title={contentItem.title}>
                          {contentItem.content}
                        </CalloutBox>
                      )}

                      {contentItem.type === 'collapsible' && (
                        <CollapsibleSection title={contentItem.title}>
                          {contentItem.content}
                        </CollapsibleSection>
                      )}
                    </div>
                  ))}

                  {/* Interactive elements */}
                  {chapter.terminal && (
                    <div className="bg-dark-bg-800 border border-dark-bg-700 rounded-lg p-4 my-4 font-mono text-sm">
                      <div className="flex items-center text-xs text-light-text-200 mb-2">
                        <span className="inline-block w-3 h-3 rounded-full bg-red-500 mr-2"></span>
                        <span className="inline-block w-3 h-3 rounded-full bg-yellow-500 mr-2"></span>
                        <span className="inline-block w-3 h-3 rounded-full bg-green-500 mr-2"></span>
                        Terminal
                      </div>
                      <pre className="text-light-text-200 whitespace-pre-wrap">
                        {chapter.terminal}
                      </pre>
                    </div>
                  )}
                </section>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default ModuleDetailPage;